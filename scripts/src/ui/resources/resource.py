#!/usr/bin/env python3
# -- coding: UTF-8 --

import os
from PyQt6.QtGui import QIcon, QPixmap

class ResourceManager:
    """Class that manages resource files"""
    
    def __init__(self):
        self.resources_dir = os.path.dirname(os.path.abspath(__file__))
        self.icons_dir = os.path.join(self.resources_dir, "icons")
        self.images_dir = os.path.join(self.resources_dir, "images")
        self.styles_dir = os.path.join(self.resources_dir, "styles")
        
        # Current theme - light theme by default
        self.current_theme = "light_theme"
        
        # Style cache
        self._style_cache = {}
    
    def get_icon(self, icon_name):
        """Return icon file as QIcon"""
        icon_path = os.path.join(self.icons_dir, icon_name)
        if os.path.exists(icon_path):
            return QIcon(icon_path)
        return QIcon()  # Empty icon
    
    def get_image(self, image_name):
        """Return image file as QPixmap"""
        image_path = os.path.join(self.images_dir, image_name)
        if os.path.exists(image_path):
            return QPixmap(image_path)
        return QPixmap()  # Empty pixmap
    
    def get_style(self, style_name):
        """Return QSS style file as string"""
        # Check from cache
        if style_name in self._style_cache:
            return self._style_cache[style_name]
            
        style_path = os.path.join(self.styles_dir, f"{style_name}.qss")
        if os.path.exists(style_path):
            with open(style_path, 'r', encoding='utf-8') as f:
                content = f.read()
                self._style_cache[style_name] = content
                return content
        return ""
    
    def get_current_theme(self):
        """Return current theme"""
        return self.get_style(self.current_theme)
    
    def set_theme(self, theme_name):
        """Change theme"""
        self.current_theme = theme_name
        return self.get_style(theme_name)
    
    def get_available_themes(self):
        """List available themes"""
        themes = []
        if os.path.exists(self.styles_dir):
            for file in os.listdir(self.styles_dir):
                if file.endswith('.qss') and file in ['light_theme.qss', 'dark_thema.qss']:
                    theme_name = file[:-4]  # Remove .qss extension
                    themes.append(theme_name)
        return themes
    
    # === NEW: Custom style methods ===
    
    def get_joint_button_style(self):
        """Return joint button style"""
        return self.get_style("joint_controls")
    
    def get_calibration_style(self, state="default"):
        """Return calibration button style"""
        styles = self.get_style("calibration_buttons")
        if state == "default":
            return ".calibration-default"
        elif state == "processing":
            return ".calibration-processing"
        elif state == "completed":
            return ".calibration-completed"
        return styles
    
    def get_toggle_style(self, active=False):
        """Return toggle button style"""
        styles = self.get_style("toggle_buttons")
        if active:
            return ".toggle-active"
        else:
            return ".toggle-inactive"
    
    def get_close_button_style(self):
        """Return close button style"""
        return self.get_style("close_button")
    
    def get_theme_button_style(self, active=False):
        """Return theme button style"""
        styles = self.get_style("theme_buttons")
        if active:
            return ".theme-button-active"
        else:
            return ".theme-button-normal"
    
    def get_label_style(self, type="header"):
        """Return label style"""
        styles = self.get_style("labels")
        if type == "header":
            return ".header-label"
        elif type == "theme":
            return ".theme-label"
        return styles

# Global resource manager instance
resource_manager = ResourceManager()