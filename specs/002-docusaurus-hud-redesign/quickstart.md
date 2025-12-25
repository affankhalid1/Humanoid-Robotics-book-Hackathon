# Quickstart: Docusaurus HUD Redesign

## Prerequisites
- Node.js 18+ installed
- npm or yarn package manager
- Git for version control

## Setup Process

1. **Install Dependencies**
   ```bash
   npm install
   # or
   yarn install
   ```

2. **Start Development Server**
   ```bash
   npm run start
   # or
   yarn start
   ```

3. **Build for Production**
   ```bash
   npm run build
   # or
   yarn build
   ```

## Key Configuration Files

- `docusaurus.config.js` - Main Docusaurus configuration
- `src/css/custom.css` - Custom CSS for HUD theme
- `src/theme/` - Custom theme components
- `src/components/` - Custom React components for HUD elements

## Custom Components

### HUD Navigation
- Located in `src/theme/Navbar/`
- Implements glassmorphism with neon-green border
- Uses JetBrains Mono font with glow effects

### Technical Module Cards
- Located in `src/components/TechnicalModule/`
- Implements dark-glass background with green borders
- Includes hover elevation and glow effects

### HUD Hero Section
- Located in `src/components/HudHero/`
- Implements system status scanner with pulsing dot
- Includes vertical scanline animation

## Theme Customization

### Color Scheme
- Background: `#050505` (obsidian)
- Accent: `#3fb950` (cyber green)
- Text: `#e0e0e0` (light gray for contrast)

### Typography
- Technical elements: JetBrains Mono
- Body text: Inter font

### Grid Pattern
- 40px blueprint radial dot grid overlay
- Implemented via CSS background pattern

## Development Workflow

1. Make changes to CSS in `src/css/custom.css`
2. Create or modify components in `src/components/`
3. Test locally with `npm run start`
4. Verify build with `npm run build`
5. Deploy changes following standard Docusaurus deployment process