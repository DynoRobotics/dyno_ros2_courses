/**
 * Naming convention validation and auto-fix
 * 
 * Enforces lowercase-with-hyphens for files and directories
 */

import { basename, dirname, join } from 'path';
import { rename, access } from 'fs/promises';
import { glob } from 'glob';
import type { ValidationError } from './types.js';

/**
 * Check if a name follows lowercase-with-hyphens convention
 */
export function isValidName(name: string): boolean {
  // Remove extension
  const nameWithoutExt = name.replace(/\.[^.]+$/, '');
  
  // Check format: lowercase, numbers, hyphens only
  return /^[a-z0-9]+(-[a-z0-9]+)*$/.test(nameWithoutExt);
}

/**
 * Convert name to lowercase-with-hyphens
 */
export function normalizeNaming(name: string): string {
  const ext = name.match(/\.[^.]+$/)?.[0] || '';
  const nameWithoutExt = name.replace(/\.[^.]+$/, '');
  
  const normalized = nameWithoutExt
    // Convert to lowercase
    .toLowerCase()
    // Replace spaces and underscores with hyphens
    .replace(/[\s_]+/g, '-')
    // Remove multiple consecutive hyphens
    .replace(/-+/g, '-')
    // Remove leading/trailing hyphens
    .replace(/^-+|-+$/g, '');
  
  return normalized + ext;
}

/**
 * Check naming conventions for all files and directories
 */
export interface NamingIssue {
  path: string;
  type: 'file' | 'directory';
  current: string;
  suggested: string;
  severity: 'error' | 'warning';
}

export async function checkNamingConventions(
  docsDir: string,
  options: {
    checkFiles?: boolean;
    checkDirectories?: boolean;
  } = {}
): Promise<NamingIssue[]> {
  const { checkFiles = true, checkDirectories = true } = options;
  const issues: NamingIssue[] = [];
  
  // Find all files and directories
  const allPaths = await glob(`${docsDir}/**/*`, {
    ignore: ['**/node_modules/**', '**/.git/**', '**/dist/**'],
    dot: false
  });
  
  for (const path of allPaths) {
    const name = basename(path);
    const isFile = path.includes('.');
    
    // Skip if not checking this type
    if (isFile && !checkFiles) continue;
    if (!isFile && !checkDirectories) continue;
    
    // Skip special directories
    if (name === '.obsidian' || name === '.git' || name === 'node_modules') {
      continue;
    }
    
    // Skip already correct names
    if (isValidName(name)) {
      continue;
    }
    
    // Found an issue
    const suggested = normalizeNaming(name);
    
    issues.push({
      path,
      type: isFile ? 'file' : 'directory',
      current: name,
      suggested,
      severity: 'warning'
    });
  }
  
  return issues;
}

/**
 * Auto-fix naming issues
 */
export interface FixResult {
  path: string;
  from: string;
  to: string;
  success: boolean;
  error?: string;
}

export async function fixNamingIssues(
  issues: NamingIssue[],
  options: {
    dryRun?: boolean;
  } = {}
): Promise<FixResult[]> {
  const { dryRun = false } = options;
  const results: FixResult[] = [];
  
  // Sort by depth (deepest first) to avoid parent/child conflicts
  const sortedIssues = [...issues].sort((a, b) => {
    const depthA = a.path.split('/').length;
    const depthB = b.path.split('/').length;
    return depthB - depthA;
  });
  
  for (const issue of sortedIssues) {
    const dir = dirname(issue.path);
    const newPath = join(dir, issue.suggested);
    
    const result: FixResult = {
      path: issue.path,
      from: issue.current,
      to: issue.suggested,
      success: false
    };
    
    if (dryRun) {
      // Just check if target exists
      try {
        await access(newPath);
        result.error = 'Target already exists';
      } catch {
        result.success = true;
      }
    } else {
      // Actually rename
      try {
        await rename(issue.path, newPath);
        result.success = true;
      } catch (error) {
        result.error = error instanceof Error ? error.message : String(error);
      }
    }
    
    results.push(result);
  }
  
  return results;
}

/**
 * Format naming issues for display
 */
export function formatNamingIssues(issues: NamingIssue[]): string {
  if (issues.length === 0) {
    return '✅ All files and directories follow naming conventions!';
  }
  
  const fileIssues = issues.filter(i => i.type === 'file');
  const dirIssues = issues.filter(i => i.type === 'directory');
  
  let output = `\n⚠️  Found ${issues.length} naming issue(s):\n\n`;
  
  if (dirIssues.length > 0) {
    output += `📁 Directories (${dirIssues.length}):\n`;
    for (const issue of dirIssues.slice(0, 10)) {
      output += `   ${issue.current} → ${issue.suggested}\n`;
      output += `     ${issue.path}\n`;
    }
    if (dirIssues.length > 10) {
      output += `   ... and ${dirIssues.length - 10} more\n`;
    }
    output += '\n';
  }
  
  if (fileIssues.length > 0) {
    output += `📄 Files (${fileIssues.length}):\n`;
    for (const issue of fileIssues.slice(0, 10)) {
      output += `   ${issue.current} → ${issue.suggested}\n`;
      output += `     ${issue.path}\n`;
    }
    if (fileIssues.length > 10) {
      output += `   ... and ${fileIssues.length - 10} more\n`;
    }
  }
  
  output += `\n💡 Run 'ros2-zenoh-tools fix-naming' to auto-fix these issues\n`;
  
  return output;
}

/**
 * Format fix results for display
 */
export function formatFixResults(results: FixResult[], dryRun: boolean): string {
  const successful = results.filter(r => r.success);
  const failed = results.filter(r => !r.success);
  
  let output = '';
  
  if (dryRun) {
    output += `\n🔍 Dry run - would rename ${successful.length} items:\n\n`;
  } else {
    output += `\n✅ Successfully renamed ${successful.length} items:\n\n`;
  }
  
  for (const result of successful.slice(0, 20)) {
    const icon = dryRun ? '  ○' : '  ✓';
    output += `${icon} ${result.from} → ${result.to}\n`;
  }
  
  if (successful.length > 20) {
    output += `  ... and ${successful.length - 20} more\n`;
  }
  
  if (failed.length > 0) {
    output += `\n❌ Failed to rename ${failed.length} items:\n\n`;
    for (const result of failed) {
      output += `  ✗ ${result.from} → ${result.to}\n`;
      output += `    Error: ${result.error}\n`;
    }
  }
  
  if (dryRun && successful.length > 0) {
    output += `\n💡 Run without --dry-run to apply these changes\n`;
  }
  
  return output;
}

