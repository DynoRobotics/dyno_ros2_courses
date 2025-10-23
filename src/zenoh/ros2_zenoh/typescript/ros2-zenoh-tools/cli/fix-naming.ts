#!/usr/bin/env node
/**
 * Auto-fix naming conventions and update wikilinks
 * 
 * Usage:
 *   npx @ros2-zenoh/tools fix-naming --check    # Dry run
 *   npx @ros2-zenoh/tools fix-naming --fix      # Apply fixes
 */

import { readFile, writeFile, rename } from 'fs/promises';
import { dirname } from 'path';
import { resolve } from 'path';
import matter from 'gray-matter';
import {
  findNamingIssues,
  buildRenameMap,
  formatRenameActions,
  updateWikilinksInContent,
  updateFrontmatterRefs,
  type RenameAction
} from '../src/naming-conventions.js';
import { glob } from 'glob';

async function main() {
  const args = process.argv.slice(2);
  
  // Resolve docs directory (can be provided as argument or default to ../../docs from tool location)
  const docsDirArg = args.find(arg => !arg.startsWith('--'));
  const docsDir = docsDirArg 
    ? resolve(process.cwd(), docsDirArg)
    : resolve(process.cwd(), '../../docs');
  
  const dryRun = args.includes('--check') || !args.includes('--fix');
  
  if (dryRun) {
    console.log('🔍 Checking naming conventions (dry run)...\n');
  } else {
    console.log('🔧 Fixing naming conventions...\n');
  }
  
  // Step 1: Find all naming issues
  console.log('📋 Step 1: Scanning for naming issues...');
  const actions = await findNamingIssues(docsDir);
  
  console.log(formatRenameActions(actions));
  
  if (actions.length === 0) {
    process.exit(0);
  }
  
  if (dryRun) {
    console.log('\n💡 Run with --fix to apply these changes');
    process.exit(0);
  }
  
  // Step 2: Build rename map
  console.log('\n📋 Step 2: Building rename map...');
  const renameMap = buildRenameMap(actions);
  console.log(`   Mapped ${renameMap.size} renames`);
  
  // Step 3: Update wikilinks in all markdown files
  console.log('\n📋 Step 3: Updating wikilinks...');
  const markdownFiles = await glob(`${docsDir}/**/*.md`);
  let totalLinkChanges = 0;
  
  for (const file of markdownFiles) {
    const content = await readFile(file, 'utf-8');
    const { updated, changes } = updateWikilinksInContent(content, renameMap);
    
    if (changes > 0) {
      await writeFile(file, updated, 'utf-8');
      console.log(`   ✅ ${file}: ${changes} links updated`);
      totalLinkChanges += changes;
    }
  }
  
  console.log(`   Updated ${totalLinkChanges} wikilinks total`);
  
  // Step 4: Update frontmatter cross-references
  console.log('\n📋 Step 4: Updating frontmatter cross-references...');
  let totalFrontmatterChanges = 0;
  
  for (const file of markdownFiles) {
    const content = await readFile(file, 'utf-8');
    const parsed = matter(content);
    
    const { updated, changes } = updateFrontmatterRefs(parsed.data, renameMap);
    
    if (changes > 0) {
      parsed.data = updated;
      const newContent = matter.stringify(parsed.content, parsed.data);
      await writeFile(file, newContent, 'utf-8');
      console.log(`   ✅ ${file}: ${changes} refs updated`);
      totalFrontmatterChanges += changes;
    }
  }
  
  console.log(`   Updated ${totalFrontmatterChanges} frontmatter refs total`);
  
  // Step 5: Perform actual renames (files first, then directories)
  console.log('\n📋 Step 5: Renaming files and directories...');
  
  // Sort: files before directories, depth-first for directories
  const sortedActions = actions.sort((a, b) => {
    if (a.type === 'file' && b.type === 'directory') return -1;
    if (a.type === 'directory' && b.type === 'file') return 1;
    return b.oldPath.length - a.oldPath.length; // Deeper paths first
  });
  
  for (const action of sortedActions) {
    try {
      await rename(action.oldPath, action.newPath);
      console.log(`   ✅ ${action.oldPath} → ${action.newPath}`);
    } catch (error) {
      console.error(`   ❌ Failed to rename ${action.oldPath}: ${error}`);
    }
  }
  
  console.log('\n✅ All fixes applied!');
  console.log('\n💡 Run validation to check for any remaining issues:');
  console.log('   npm run validate');
}

main().catch(error => {
  console.error('❌ Error:', error);
  process.exit(1);
});

