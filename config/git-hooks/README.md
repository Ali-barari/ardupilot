# Avestec ArduPilot Git Hooks

Same hooks and versioning scheme as the AvesAID (PX4) repo, adapted for ArduPilot:
the upstream base comes from ArduPilot's per-vehicle release tags (`Rover-4.6.3`),
so tags look like `v4.6.3-1.2.3`.

This directory contains Git hooks. The pre-commit hook installs them into `.git/hooks` on the first commit.

## Available Hooks

### post-merge
- **Purpose**: Tags `develop` when a feature branch is fast-forward merged into it
- **Behavior**: One tag on the new `develop` HEAD; the bump is the highest keyword found in all merged commit subjects. Skips if HEAD is already tagged (e.g. `git pull` of an already tagged develop).

### post-commit
- **Purpose**: Automatic versioning system (also runs the post-merge versioning)
- **Behavior**: Creates version tags in format `v{upstream_version}-{avesaid_major}.{avesaid_minor}.{avesaid_patch}`
- **Branches**: Only `develop` is tagged. Feature branches are never tagged.
- **Git Configuration**: Automatically configures:
  - `merge.ff only` - Forces fast-forward merges to prevent unwanted merge commits
  - `push.followTags true` - Automatically pushes tags with commits
  - `remote.origin.tagopt --tags` - Automatically fetches all tags

### pre-commit
- **Purpose**: Pre-commit validation and checks
- **Behavior**: Runs before each commit to validate code quality

### post-checkout
- **Purpose**: Post-checkout operations
- **Behavior**: Runs after branch checkout operations

## Installation

To install all hooks by hand:

```bash
cp config/git-hooks/{pre-commit,post-commit,post-checkout,post-merge} .git/hooks/
chmod +x .git/hooks/{pre-commit,post-commit,post-checkout,post-merge}
```

## Version Tagging System

The post-commit hook implements automatic versioning:

### Tag Format
`v{upstream_version}-{avesaid_major}.{avesaid_minor}.{avesaid_patch}`

Examples:
- `v4.6.3-1.2.3` - Based on ArduPilot Rover-4.6.3, Avestec version 1.2.3
- `v4.6.3-2.0.0` - Major version increment

### Version Increment Rules
- `[major]` or `BREAKING CHANGE` → increment major, reset minor/patch to 0
- `[minor]` or `feat:` → increment minor, reset patch to 0
- Default → increment patch

### Branch Behavior
- **develop**: Auto-versioning enabled (direct commits and fast-forward merges)
- **feature branches**: Never tagged; they get a version when merged into develop
- **main**: Auto-versioning disabled (manual tagging only)

### Merging a feature
```bash
git checkout feature/my-thing && git rebase develop
git checkout develop && git merge feature/my-thing   # fast-forward -> new tag
```

## Git Workflow Configuration

The hooks automatically configure Git to prevent merge commit issues:

### Fast-Forward Only Merges
```bash
git config merge.ff only
```
- Prevents unwanted merge commits
- Forces proper conflict resolution via rebase

### Automatic Tag Handling
```bash
git config push.followTags true
git config remote.origin.tagopt --tags
```
- Tags are automatically pushed/fetched with commits
- Ensures version tags stay synchronized

## Troubleshooting

### "Not possible to fast-forward, aborting"
This is expected behavior when branches have diverged. Resolve with:

```bash
# Option 1: Rebase develop onto main
git checkout develop
git rebase main
git checkout main
git merge develop  # Should fast-forward cleanly

# Option 2: Reset main to match remote
git checkout main
git reset --hard origin/main
git merge develop  # Should fast-forward cleanly
```

### Disable Hooks Temporarily
```bash
# Remove execute permission
chmod -x .git/hooks/post-commit

# Or rename the hook
mv .git/hooks/post-commit .git/hooks/post-commit.disabled
```

## Team Setup

For new team members or fresh clones:

1. Clone the repository
2. Install the hooks (see Installation)
3. Make first commit to trigger hook configuration

The hooks will ensure consistent Git behavior across all team members.
