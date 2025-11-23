"""
Generate LED beacon effect GIF animations for ESP32 web interface.
Creates Gradient_Fill.gif and Pulsing_Dot.gif optimized for embedded systems.
"""

from PIL import Image, ImageDraw
import math

def create_gradient_fill_gif(filename='data/Gradient_Fill.gif', 
                             width=480, height=480, 
                             num_leds=30, frames=60):
    """
    Create a gradient fill animation showing LEDs filling from start to end
    with a smooth color gradient from green to red.
    """
    images = []
    led_height = height // num_leds
    led_spacing = 2
    
    for frame in range(frames):
        # Create new frame
        img = Image.new('RGB', (width, height), color='black')
        draw = ImageDraw.Draw(img)
        
        # Calculate how many LEDs are lit (fill animation)
        progress = (frame + 1) / frames
        lit_leds = int(num_leds * progress)
        
        for i in range(lit_leds):
            # Calculate color gradient from green to red
            color_progress = i / (num_leds - 1) if num_leds > 1 else 0
            r = int(255 * color_progress)
            g = int(255 * (1 - color_progress))
            b = 0
            color = (r, g, b)
            
            # Draw LED bar
            y_start = height - (i + 1) * led_height + led_spacing
            y_end = height - i * led_height - led_spacing
            
            draw.rectangle(
                [(50, y_start), (width - 50, y_end)],
                fill=color,
                outline=None
            )
        
        images.append(img)
    
    # Save as optimized GIF
    images[0].save(
        filename,
        save_all=True,
        append_images=images[1:],
        duration=50,  # 50ms per frame = 20fps
        loop=0,
        optimize=True
    )
    print(f"✓ Created {filename} ({len(images)} frames)")


def create_pulsing_dot_gif(filename='data/Pulsing_Dot.gif',
                           width=480, height=480,
                           num_leds=30, frames=60):
    """
    Create a pulsing/breathing dot animation showing a single LED that
    brightens and dims in a smooth breathing pattern.
    """
    images = []
    led_height = height // num_leds
    led_spacing = 2
    dot_position = num_leds // 2  # Middle of the strip
    
    for frame in range(frames):
        # Create new frame
        img = Image.new('RGB', (width, height), color='black')
        draw = ImageDraw.Draw(img)
        
        # Calculate breathing intensity (sine wave for smooth pulse)
        pulse_progress = frame / frames
        intensity = (math.sin(pulse_progress * 2 * math.pi) + 1) / 2  # 0 to 1
        
        # Minimum brightness so dot is always visible
        min_brightness = 0.2
        brightness = min_brightness + (1 - min_brightness) * intensity
        
        # Color: blue with varying brightness
        r = int(0 * brightness)
        g = int(100 * brightness)
        b = int(255 * brightness)
        color = (r, g, b)
        
        # Draw the pulsing LED
        y_start = height - (dot_position + 1) * led_height + led_spacing
        y_end = height - dot_position * led_height - led_spacing
        
        draw.rectangle(
            [(50, y_start), (width - 50, y_end)],
            fill=color,
            outline=None
        )
        
        # Add glow effect around the dot
        if brightness > 0.5:
            glow_intensity = int(50 * (brightness - 0.5) * 2)
            glow_color = (0, int(50 * (brightness - 0.5) * 2), int(128 * (brightness - 0.5) * 2))
            
            # Draw glow above and below
            if dot_position > 0:
                y_glow_above = height - (dot_position + 2) * led_height + led_spacing
                y_glow_above_end = height - (dot_position + 1) * led_height - led_spacing
                draw.rectangle(
                    [(50, y_glow_above), (width - 50, y_glow_above_end)],
                    fill=glow_color,
                    outline=None
                )
            
            if dot_position < num_leds - 1:
                y_glow_below = height - dot_position * led_height + led_spacing
                y_glow_below_end = height - (dot_position - 1) * led_height - led_spacing
                draw.rectangle(
                    [(50, y_glow_below), (width - 50, y_glow_below_end)],
                    fill=glow_color,
                    outline=None
                )
        
        images.append(img)
    
    # Save as optimized GIF
    images[0].save(
        filename,
        save_all=True,
        append_images=images[1:],
        duration=50,  # 50ms per frame = 20fps
        loop=0,
        optimize=True
    )
    print(f"✓ Created {filename} ({len(images)} frames)")


def create_progress_bar_gif(filename='data/Progress_Bar.gif',
                            width=480, height=480,
                            num_leds=30, frames=60):
    """
    Create a progress bar animation showing LEDs filling from bottom to top
    with a white climbing dot that continuously moves up.
    """
    images = []
    led_height = height // num_leds
    led_spacing = 2
    
    for frame in range(frames):
        # Create new frame
        img = Image.new('RGB', (width, height), color='black')
        draw = ImageDraw.Draw(img)
        
        # Calculate how many LEDs are lit (progress animation)
        progress = (frame + 1) / frames
        lit_leds = int(num_leds * progress)
        
        # Climbing dot position (continuous loop)
        climb_position = int((frame * 2) % num_leds)  # Climbs faster than fill
        
        # Green color for filled progress
        progress_color = (0, 255, 0)
        # White color for climbing dot
        climb_color = (255, 255, 255)
        
        for i in range(num_leds):
            y_start = height - (i + 1) * led_height + led_spacing
            y_end = height - i * led_height - led_spacing
            
            if i == climb_position:
                # Draw climbing dot (always visible, bright white)
                draw.rectangle(
                    [(50, y_start), (width - 50, y_end)],
                    fill=climb_color,
                    outline=None
                )
            elif i < lit_leds:
                # Draw filled progress bar (green)
                draw.rectangle(
                    [(50, y_start), (width - 50, y_end)],
                    fill=progress_color,
                    outline=None
                )
        
        images.append(img)
    
    # Save as optimized GIF
    images[0].save(
        filename,
        save_all=True,
        append_images=images[1:],
        duration=50,  # 50ms per frame = 20fps
        loop=0,
        optimize=True
    )
    print(f"✓ Created {filename} ({len(images)} frames)")


def create_climbing_dot_gif(filename='data/Climbing_Dot.gif',
                            width=480, height=480,
                            num_leds=30, frames=60):
    """
    Create a climbing dot animation showing a single LED moving up the strip.
    """
    images = []
    led_height = height // num_leds
    led_spacing = 2
    
    for frame in range(frames):
        # Create new frame
        img = Image.new('RGB', (width, height), color='black')
        draw = ImageDraw.Draw(img)
        
        # Calculate dot position (climbs from bottom to top)
        progress = frame / frames
        dot_position = int(num_leds * progress) % num_leds
        
        # White color for the dot
        color = (255, 255, 255)
        
        # Draw the climbing LED
        y_start = height - (dot_position + 1) * led_height + led_spacing
        y_end = height - dot_position * led_height - led_spacing
        
        draw.rectangle(
            [(50, y_start), (width - 50, y_end)],
            fill=color,
            outline=None
        )
        
        # Add trail effect (dimmer LEDs behind)
        trail_length = 3
        for i in range(1, trail_length + 1):
            if dot_position - i >= 0:
                trail_brightness = 1.0 - (i / (trail_length + 1))
                trail_color = (
                    int(255 * trail_brightness),
                    int(255 * trail_brightness),
                    int(255 * trail_brightness)
                )
                
                y_trail_start = height - (dot_position - i + 1) * led_height + led_spacing
                y_trail_end = height - (dot_position - i) * led_height - led_spacing
                
                draw.rectangle(
                    [(50, y_trail_start), (width - 50, y_trail_end)],
                    fill=trail_color,
                    outline=None
                )
        
        images.append(img)
    
    # Save as optimized GIF
    images[0].save(
        filename,
        save_all=True,
        append_images=images[1:],
        duration=50,  # 50ms per frame = 20fps
        loop=0,
        optimize=True
    )
    print(f"✓ Created {filename} ({len(images)} frames)")


if __name__ == '__main__':
    print("Generating LED beacon effect animations...")
    print("=" * 50)
    
    # Generate all 4 animations
    create_progress_bar_gif()
    create_climbing_dot_gif()
    create_gradient_fill_gif()
    create_pulsing_dot_gif()
    
    print("=" * 50)
    print("Done! Check the 'data' folder for the generated GIFs.")
    print("\nNext steps:")
    print("1. Check file sizes (should be < 250KB each)")
    print("2. If too large, reduce frames or dimensions")
    print("3. Upload to ESP32 with: pio run --target uploadfs")
