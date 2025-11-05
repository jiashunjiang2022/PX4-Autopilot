# Version Information

## Current Version

**Branch**: `pid`  
**Latest Commit**: `4b22ca6f71`  
**Status**: Production Ready ✅

## Quick Access

- **GitHub**: https://github.com/jiashunjiang2022/PX4-Autopilot/tree/pid
- **Summary**: See `FIXES_SUMMARY.md` for detailed fix descriptions

## How to Use This Version

### Clone and Build

```bash
git clone https://github.com/jiashunjiang2022/PX4-Autopilot.git
cd PX4-Autopilot
git checkout pid
make px4_sitl_default
```

### Test in Simulation

```bash
make px4_sitl gazebo-classic
# In QGroundControl, set FW_GUIDANCE_MODE = 1 for L1, or 0 for NPFG
```

### Revert to Previous Version

```bash
# View commit history
git log --oneline -20

# Revert to specific commit
git reset --hard <commit_hash>

# Force push (use with caution!)
git push origin pid --force
```

## Key Commits

1. **4b22ca6f71** - Add comprehensive fixes summary (current)
2. **365914536f** - Production ready: Remove all debugging
3. **731b35b848** - Clean up debugging messages
4. **cdb6c00d89** - FINAL FIX: TECS re-initialization
5. **47dd56ecc7** - L1 course calculation fix (eta1 only)
6. **60697ec2e1** - L1 lateral acceleration limit

## All Issues Resolved

- ✅ Altitude control works correctly in AUTO mode
- ✅ TECS properly initialized after takeoff  
- ✅ L1 path following smooth and accurate
- ✅ No S-shaped oscillations
- ✅ Cross-track error < 10m on straight segments
- ✅ Multi-waypoint missions execute successfully

## Tested Scenarios

- ✅ Takeoff → Multiple waypoints → Landing
- ✅ L1 guidance mode (FW_GUIDANCE_MODE=1)
- ✅ NPFG guidance mode (FW_GUIDANCE_MODE=0)
- ✅ Waypoints at various altitudes (80-150ft AGL tested)
- ✅ Path following with heading changes up to 170°

---

**Last Updated**: 2025-10-17  
**Maintainer**: jiashunjiang2022







