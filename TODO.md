# Backend Availability Check Implementation Plan
Status: In Progress

## Steps:
- [x] 1. Understand codebase and create detailed edit plan (completed).
- [x] 2. Edit `src/Backend_Manager.cpp`: Add `backendCheckAvailability()` and update `backendTask()` (done).
- [x] 3. Verify build: Run `pio run` (user to run/test).
- [ ] 4. Test on device: `pio run -t upload`, monitor Serial for check post-NTP/OTA, no immediate JSON POST, correct LED.
- [ ] 5. Confirm task complete, attempt_completion.

**Next step:** Implement edit #2.

