# AI Usage Guide for memo-dec

This guide helps AI agents understand how to use memo-dec context files.

## Directory Structure

```
.memo/
├── .memoignore          # Files/folders to ignore
├── memosymbols.txt      # Code symbols (functions, classes, variables)
├── memocontent.json     # File summaries with hashes
├── memotree/
│   ├── memofoldertree.txt   # Folder structure only
│   └── memofiletree.txt     # Full file tree with sizes
├── memodocs/            # Additional documentation
├── .memosymbols-hist/   # Symbol history (timestamped JSON)
└── .memocontent-hist/   # Content history (timestamped JSON)
```

## Key Files

### memosymbols.txt
Ultra-compact symbol format:
```
path/to/file.py
  10:fun:function_name  25:cls:ClassName  40:var:VARIABLE_NAME
```
Format: `line:type:name` where type is:
- `fun` = function
- `cls` = class
- `var` = variable
- `com` = component (React/Vue)

### memocontent.json
JSON format with file metadata:
```json
{
  "path/to/file.py": {
    "hash": "abc123...",
    "last_updated": 1234567890,
    "summary": "File description..."
  }
}
```

## CLI Commands for AI Agents

```bash
# Quick queries
memo-dec getsymbols json .py           # Python symbols as JSON
memo-dec getsummary markdown .js src/  # JS summaries from src/

# Updates
memo-dec update --symbols              # Re-extract symbols (with backup)
memo-dec update --content              # Incremental content update
memo-dec update --all                  # Update both

# Maintenance
memo-dec addignore "*.log" "temp/"     # Add ignore patterns
memo-dec findignore                    # AI-powered ignore generation
```

## Best Practices

1. **Before major refactoring**: Run `memo-dec update --all` to refresh context
2. **Adding new file types**: Update `.memo/.memoignore` to include/exclude
3. **Large projects**: Use `getsymbols` with filters to query specific files
4. **History tracking**: Check `.memosymbols-hist/` for previous states

## Integration Tips

- Symbols file is optimized for fast parsing (100+ symbols per line)
- Use `--json` output for programmatic access
- File hashes in memocontent.json enable change detection
- Tree files help understand project structure at a glance
