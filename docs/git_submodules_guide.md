# Git Submodules Guide for Riggu ROS2 Workspace

This guide provides comprehensive information about managing git submodules in the Riggu ROS2 workspace. The project uses git submodules to include external ROS2 packages and maintain version control across different repositories.

## 📁 Current Submodules in Riggu Workspace

The following packages are managed as git submodules:

```
src/
├── rplidar_ros/                    # Slamtec RPLidar driver
├── riggu_description/              # Robot URDF and description files
├── twist_stamper/                  # Twist message stamping utility
└── drivelink_ros_interface/        # Hardware interface for drive system
```

### Submodule Details

| Submodule | Repository | Purpose |
|-----------|------------|---------|
| `src/rplidar_ros` | [Slamtec/rplidar_ros](https://github.com/Slamtec/rplidar_ros.git) | Official driver for RPLidar sensors |
| `src/riggu_description` | [taruntom1/riggu_description](https://github.com/taruntom1/riggu_description.git) | Robot description, URDF, meshes |
| `src/twist_stamper` | [taruntom1/twist_stamper](https://github.com/taruntom1/twist_stamper.git) | Utility for stamping twist messages |
| `src/drivelink_ros_interface` | [taruntom1/drivelink_ros_interface](https://github.com/taruntom1/drivelink_ros_interface.git) | Hardware interface for drive system |

## 🚀 Quick Reference Commands

### Initial Setup (First Time Contributors)

```bash
# Clone the main repository with all submodules
git clone --recursive https://github.com/your-org/riggu_ws.git

# If you already cloned without --recursive
git submodule update --init --recursive
```

### Daily Development Workflow

```bash
# Update all submodules to latest commits referenced by main repo
git submodule update --recursive

# Pull latest changes from main repo and update submodules
git pull && git submodule update --recursive

# Check status of all submodules
git submodule status
```

## 📖 Detailed Operations

### 1. Working with Existing Submodules

#### Updating Submodules to Latest Versions

```bash
# Update a specific submodule to its latest commit
cd src/rplidar_ros
git pull origin main  # or master, depending on default branch
cd ../..
git add src/rplidar_ros
git commit -m "Update rplidar_ros submodule to latest version"

# Update all submodules to their latest commits
git submodule update --remote --recursive
git add .
git commit -m "Update all submodules to latest versions"
```

#### Checking Submodule Status

```bash
# See which commit each submodule is currently on
git submodule status

# See if submodules have uncommitted changes
git submodule foreach git status

# See detailed information about submodules
git submodule summary
```

### 2. Making Changes to Submodules

#### Modifying Code in Submodules

```bash
# Navigate to the submodule
cd src/riggu_description

# Create a new branch for your changes
git checkout -b feature/update-robot-model

# Make your changes and commit them
# ... edit files ...
git add .
git commit -m "Update robot model with new sensors"

# Push the changes to the submodule's repository
git push origin feature/update-robot-model

# Go back to main workspace and update the submodule reference
cd ../..
git add src/riggu_description
git commit -m "Update riggu_description submodule with new robot model"
git push
```

#### Working on a Specific Branch of a Submodule

```bash
# Configure submodule to track a specific branch
git config -f .gitmodules submodule.src/riggu_description.branch develop

# Update submodule to track the specified branch
git submodule update --remote src/riggu_description

# Commit the configuration change
git add .gitmodules src/riggu_description
git commit -m "Configure riggu_description to track develop branch"
```

### 3. Adding New Submodules

```bash
# Add a new ROS2 package as a submodule
git submodule add https://github.com/author/new_ros_package.git src/new_ros_package

# Commit the new submodule
git commit -m "Add new_ros_package as submodule"

# Push the changes
git push
```

### 4. Removing Submodules

```bash
# Remove a submodule (requires multiple steps)
git submodule deinit src/old_package
git rm src/old_package
rm -rf .git/modules/src/old_package
git commit -m "Remove old_package submodule"
```

## 🔧 Advanced Operations

### Working with Detached HEAD State

When you run `git submodule update`, submodules enter a "detached HEAD" state. This is normal for read-only usage, but if you need to make changes:

```bash
# Navigate to submodule and checkout a branch
cd src/riggu_description
git checkout main  # or the appropriate branch

# Make changes and commit
# ... edit files ...
git add .
git commit -m "Your changes"

# Push changes
git push origin main

# Update parent repository reference
cd ../..
git add src/riggu_description
git commit -m "Update riggu_description submodule reference"
```

### Handling Merge Conflicts in Submodules

```bash
# If you encounter conflicts during git pull
git pull
# Auto-merging may fail for submodules

# Update submodules manually
git submodule update --recursive

# If conflicts persist, resolve manually:
cd src/problematic_submodule
git status
# Resolve conflicts and commit
git add .
git commit
cd ../..
git add src/problematic_submodule
git commit -m "Resolve submodule conflicts"
```

### Parallel Operations on All Submodules

```bash
# Run a command in all submodules
git submodule foreach 'git status'
git submodule foreach 'git fetch'
git submodule foreach 'git checkout main'

# More complex operations
git submodule foreach 'git pull origin $(git symbolic-ref --short HEAD) || echo "Failed to pull $name"'
```

## 🛠️ ROS2-Specific Considerations

### Building After Submodule Updates

```bash
# Always rebuild after updating submodules
git submodule update --recursive
source /opt/ros/kilted/setup.bash
colcon build
source install/setup.bash
```

### Package Dependencies

When adding new submodules that are ROS2 packages:

1. **Check dependencies**: Ensure the new package's dependencies are available
2. **Update package.xml**: If needed, update dependency lists in your packages
3. **Test build**: Always test the build after adding new submodules

```bash
# Check for missing dependencies
rosdep check --from-paths src --ignore-src -r

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y
```

## 🚨 Common Issues and Solutions

### Issue 1: Submodule Directory is Empty

```bash
# Solution: Initialize and update submodules
git submodule update --init --recursive
```

### Issue 2: Submodule Shows Uncommitted Changes

```bash
# Check what changed
cd src/problematic_submodule
git diff

# If you don't want the changes
git checkout .

# If you want to keep changes
git add .
git commit -m "Local changes to submodule"
```

### Issue 3: Cannot Pull Due to Submodule Conflicts

```bash
# Reset submodules to their committed state
git submodule update --init --force --recursive

# Then try pulling again
git pull
```

### Issue 4: Submodule Points to Non-existent Commit

```bash
# Update to latest available commit
cd src/problematic_submodule
git fetch
git checkout origin/main  # or appropriate branch
cd ../..
git add src/problematic_submodule
git commit -m "Update submodule to available commit"
```

## 📋 Best Practices for Riggu Development

### 1. **Always Use Recursive Operations**
```bash
# Good
git submodule update --recursive
git clone --recursive

# Avoid
git submodule update  # misses nested submodules
```

### 2. **Regular Submodule Maintenance**
```bash
# Weekly maintenance routine
git pull
git submodule update --recursive
colcon build  # ensure everything still builds
```

### 3. **Branch Management**
- Keep submodules on stable branches/tags for releases
- Use development branches only for active development
- Document which branch each submodule should track

### 4. **Testing Before Commits**
```bash
# Always test after submodule changes
git submodule update --recursive
source /opt/ros/kilted/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
# Run your tests here
```

### 5. **Communication with Team**
- Always communicate when updating submodules
- Document the reason for submodule updates
- Test in simulation before pushing submodule updates

## 🔍 Troubleshooting Commands

```bash
# Get detailed submodule information
git submodule status --recursive

# Check for uncommitted changes in all submodules
git submodule foreach git status

# Reset everything to clean state
git submodule foreach git clean -fd
git submodule foreach git reset --hard HEAD

# Check which submodules have updates available
git submodule foreach git fetch
git submodule foreach 'git log --oneline HEAD..origin/$(git symbolic-ref --short HEAD)'
```

## 📚 Additional Resources

- [Git Submodules Official Documentation](https://git-scm.com/book/en/v2/Git-Tools-Submodules)
- [ROS2 Package Management Best Practices](https://docs.ros.org/en/kilted/Tutorials.html)
- [Colcon Build System Documentation](https://colcon.readthedocs.io/)

## 🤝 Contributing

When contributing to this project:

1. **Fork the main repository and relevant submodules**
2. **Create feature branches in both main repo and submodules**
3. **Test thoroughly with the build system**
4. **Submit pull requests to submodules first**
5. **Update main repository after submodule PRs are merged**

For questions about submodule management in this project, please open an issue in the main repository.
