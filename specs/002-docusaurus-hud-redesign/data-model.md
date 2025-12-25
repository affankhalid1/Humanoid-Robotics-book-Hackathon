# Data Model: Docusaurus HUD Redesign

## Theme Configuration

### HUD Theme Entity
- **name**: String (HUD Theme)
- **primaryColor**: String (#3fb950 - cyber green)
- **backgroundColor**: String (#050505 - deep obsidian)
- **gridSize**: Number (40 - px for blueprint radial dot grid)
- **glassOpacity**: Number (0.1 - for glassmorphism effects)
- **fontPrimary**: String ("JetBrains Mono" - technical elements)
- **fontSecondary**: String ("Inter" - body text)

## Component Specifications

### Technical Module Card
- **id**: String (unique identifier)
- **title**: String (module name)
- **label**: String (formatted as "XX // MODULE_NAME")
- **description**: String (technical description)
- **icon**: String (optional icon reference)
- **isActive**: Boolean (for hover effects)

### Navigation Item
- **id**: String (unique identifier)
- **label**: String (menu item text)
- **url**: String (navigation path)
- **isActive**: Boolean (current page indicator)
- **isGlowing**: Boolean (active state glow effect)

### Hero Section
- **title**: String ("PHYSICAL AI")
- **statusText**: String ("SYSTEM ONLINE: PROTOCOL_HUMANOID_V1")
- **isPulsing**: Boolean (pulsing dot animation state)
- **scanlineActive**: Boolean (vertical scanline animation state)
- **primaryButtonLabel**: String (call-to-action text)
- **secondaryButtonLabel**: String (alternative action text)

## CSS Custom Properties

### Color Variables
- **--hud-bg-obsidian**: #050505
- **--hud-accent-green**: #3fb950
- **--hud-text-primary**: #e0e0e0
- **--hud-text-secondary**: #a0a0a0

### Typography Variables
- **--hud-font-mono**: "JetBrains Mono", monospace
- **--hud-font-sans**: "Inter", sans-serif

### Effect Variables
- **--hud-glass-blur**: 12px
- **--hud-glow-strength**: 8px