# Contributing to Bambu LED Controller

Thank you for considering contributing to this project! Here's how you can help.

## 🐛 Reporting Bugs

Before creating a bug report, please check existing issues to avoid duplicates.

**When reporting a bug, include:**
- ESP32 board model and specifications
- LED strip type and count
- Bambu printer model (X1C, P1P, etc.)
- Firmware version
- Serial monitor output showing the error
- Steps to reproduce
- Expected vs actual behavior

## 💡 Suggesting Features

Feature requests are welcome! Please:
- Check if the feature already exists or is planned
- Describe the use case clearly
- Explain why this feature would benefit other users
- Consider implementation complexity

## 🔧 Pull Requests

### Before You Start

1. **Fork the repository**
2. **Create a feature branch** from `main`:
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. **Discuss major changes** by opening an issue first

### Code Standards

- **Code Style**: Follow existing code formatting
- **Comments**: Add comments for complex logic
- **Memory Safety**: Avoid memory leaks, use appropriate data types
- **Testing**: Test thoroughly on real hardware before submitting

### Commit Messages

Use clear, descriptive commit messages:
```
✅ Good:
- "Add auto-off timer feature"
- "Fix MQTT reconnection bug"
- "Update README with setup instructions"

❌ Bad:
- "Update"
- "Fix stuff"
- "Changes"
```

### Pull Request Process

1. **Update README** if adding features
2. **Test on hardware** - include test results in PR description
3. **Check for credentials** - ensure no personal WiFi/MQTT info
4. **Document changes** - add comments explaining new code
5. **Create PR** with clear description:
   - What does it do?
   - Why is it needed?
   - How did you test it?

## 🏗️ Development Setup

```bash
# Clone your fork
git clone https://github.com/YOUR_USERNAME/bambu-led-controller.git
cd bambu-led-controller

# Install PlatformIO
# (Via VS Code extension or CLI)

# Build the project
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio device monitor
```

## 📝 Documentation

Improvements to documentation are always welcome:
- Fix typos or unclear instructions
- Add diagrams or photos
- Expand setup guides
- Translate to other languages

## 🧪 Testing

When testing changes:
- Test all LED modes (Auto, Manual, Printer, Off)
- Test web interface on multiple devices
- Test MQTT connection and reconnection
- Test OTA updates
- Monitor for memory leaks during long runs
- Check serial output for errors

## ⚠️ Security

**Never commit:**
- WiFi passwords
- MQTT access codes
- Device serial numbers
- IP addresses from your network

If you accidentally commit credentials:
- Don't just remove them in a new commit
- Contact maintainers immediately
- Consider the credentials compromised

## 🎨 Areas for Contribution

Ideas for contributions:
- **LED Effects**: Add new animation patterns
- **Web UI**: Improve design or add features
- **MQTT**: Add support for more printer data
- **Documentation**: Photos, diagrams, translations
- **Hardware**: Test with different ESP32 boards
- **Integration**: Home Assistant, MQTT discovery, etc.
- **Testing**: Add automated tests
- **Performance**: Optimize memory usage or speed

## 📞 Getting Help

- **Questions?** Open a [Discussion](../../discussions)
- **Bug?** Open an [Issue](../../issues)
- **Chat?** Check if there's a Discord/Slack community

## 📜 License

By contributing, you agree that your contributions will be licensed under the same MIT License that covers this project.

---

**Thank you for helping make this project better! 🎉**
