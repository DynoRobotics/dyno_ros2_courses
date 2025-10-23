/**
 * Naming convention enforcement and auto-fixing
 */

import { readdir, stat, readFile, writeFile, rename } from 'fs/promises';
import { join, dirname, basename } from 'path';

export interface RenameAction {
  oldPath: string;
  newPath: string;
  type: 'file' | 'directory';
}

export interface WikilinkUpdate {
  file: string;
  oldLink: string;
  newLink: string;
  lineNumber?: number;
}

/**
 * Convert name to lowercase-with-hyphens convention
 */
export function toConventionName(name: string): string {
  return name
    .replace(/\.md$/, '')  // Remove extension temporarily
    .replace(/([A-Z])/g, '-$1')  // Insert hyphen before capitals
    .toLowerCase()
    .replace(/^-/, '')  // Remove leading hyphen
    .replace(/--+/g, '-')  // Collapse multiple hyphens
    .replace(/_/g, '-')  // Replace underscores with hyphens
    + (name.endsWith('.md') ? '.md' : '');
}

/**
 * Check if a name follows the convention
 */
export function isConventionalName(name: string): boolean {
  const baseName = name.replace(/\.md$/, '');
  
  // Allow numbers at start (for ADRs like "001-name.md")
  // Allow lowercase letters, numbers, and hyphens
  return /^[0-9a-z]+(-[0-9a-z]+)*$/.test(baseName);
}

/**
 * Scan directory recursively and find files/dirs that need renaming
 */
export async function findNamingIssues(
  rootDir: string,
  ignorePatterns: string[] = ['node_modules', '.git', 'dist', '_inbox']
): Promise<RenameAction[]> {
  const actions: RenameAction[] = [];
  
  async function scan(dir: string, relativePath: string = ''): Promise<void> {
    const entries = await readdir(dir, { withFileTypes: true });
    
    for (const entry of entries) {
      const fullPath = join(dir, entry.name);
      const relPath = join(relativePath, entry.name);
      
      // Skip ignored patterns
      if (ignorePatterns.some(pattern => relPath.includes(pattern))) {
        continue;
      }
      
      // Check if name needs fixing
      if (!isConventionalName(entry.name)) {
        const newName = toConventionName(entry.name);
        const newPath = join(dirname(fullPath), newName);
        
        actions.push({
          oldPath: fullPath,
          newPath,
          type: entry.isDirectory() ? 'directory' : 'file'
        });
      }
      
      // Recurse into directories
      if (entry.isDirectory()) {
        await scan(fullPath, relPath);
      }
    }
  }
  
  await scan(rootDir);
  return actions;
}

/**
 * Extract all wikilinks from markdown content
 */
export function extractWikilinks(content: string): Array<{ link: string; display?: string; start: number; end: number }> {
  const regex = /\[\[([^\]|]+)(?:\|([^\]]+))?\]\]/g;
  const links: Array<{ link: string; display?: string; start: number; end: number }> = [];
  
  let match;
  while ((match = regex.exec(content)) !== null) {
    links.push({
      link: match[1],
      display: match[2],
      start: match.index,
      end: match.index + match[0].length
    });
  }
  
  return links;
}

/**
 * Update wikilinks in content based on rename map
 */
export function updateWikilinksInContent(
  content: string,
  renameMap: Map<string, string>
): { updated: string; changes: number } {
  const links = extractWikilinks(content);
  let updated = content;
  let changes = 0;
  
  // Process links in reverse order to maintain string indices
  for (let i = links.length - 1; i >= 0; i--) {
    const { link, display, start, end } = links[i];
    
    // Check if this link needs updating
    const linkBasename = basename(link, '.md');
    const newName = renameMap.get(linkBasename);
    
    if (newName && newName !== linkBasename) {
      // Construct new wikilink
      const newLink = link.includes('/') 
        ? link.replace(linkBasename, newName)
        : newName;
      
      const newWikilink = display 
        ? `[[${newLink}|${display}]]`
        : `[[${newLink}]]`;
      
      updated = updated.substring(0, start) + newWikilink + updated.substring(end);
      changes++;
    }
  }
  
  return { updated, changes };
}

/**
 * Update frontmatter cross-references based on rename map
 */
export function updateFrontmatterRefs(
  frontmatter: any,
  renameMap: Map<string, string>
): { updated: any; changes: number } {
  const updated = { ...frontmatter };
  let changes = 0;
  
  // Fields that contain cross-references
  const refFields = [
    'related_specs',
    'related_adrs',
    'related_patterns',
    'related_conventions'
  ];
  
  for (const field of refFields) {
    if (Array.isArray(updated[field])) {
      updated[field] = updated[field].map((ref: string) => {
        const newName = renameMap.get(ref);
        if (newName && newName !== ref) {
          changes++;
          return newName;
        }
        return ref;
      });
    }
  }
  
  // Handle depends_on nested structure
  if (updated.depends_on) {
    for (const key of ['specs', 'adrs', 'patterns', 'conventions']) {
      if (Array.isArray(updated.depends_on[key])) {
        updated.depends_on[key] = updated.depends_on[key].map((ref: string) => {
          const newName = renameMap.get(ref);
          if (newName && newName !== ref) {
            changes++;
            return newName;
          }
          return ref;
        });
      }
    }
  }
  
  return { updated, changes };
}

/**
 * Build rename map: old basename -> new basename
 */
export function buildRenameMap(actions: RenameAction[]): Map<string, string> {
  const map = new Map<string, string>();
  
  for (const action of actions) {
    if (action.type === 'file') {
      const oldBase = basename(action.oldPath, '.md');
      const newBase = basename(action.newPath, '.md');
      map.set(oldBase, newBase);
    }
  }
  
  return map;
}

/**
 * Format rename actions for display
 */
export function formatRenameActions(actions: RenameAction[]): string {
  if (actions.length === 0) {
    return '✅ All names follow conventions!';
  }
  
  let output = `\n📝 Found ${actions.length} items to rename:\n\n`;
  
  const files = actions.filter(a => a.type === 'file');
  const dirs = actions.filter(a => a.type === 'directory');
  
  if (dirs.length > 0) {
    output += `📁 Directories (${dirs.length}):\n`;
    for (const action of dirs) {
      output += `  ${basename(action.oldPath)} → ${basename(action.newPath)}\n`;
    }
    output += '\n';
  }
  
  if (files.length > 0) {
    output += `📄 Files (${files.length}):\n`;
    for (const action of files) {
      output += `  ${basename(action.oldPath)} → ${basename(action.newPath)}\n`;
    }
  }
  
  return output;
}

