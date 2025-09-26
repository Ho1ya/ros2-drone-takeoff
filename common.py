#!/usr/bin/env python3
"""
Шаблон со всеми необходимыми импортами для проекта дрона
"""

# Основные библиотеки Python
import sys
import os
import time
import math
import asyncio
from typing import List, Dict, Tuple, Optional, Any

# Компьютерное зрение и обработка изображений
import cv2
import numpy as np
from scipy import spatial
from scipy.optimize import minimize

# Обработка изображений и визуализация
from PIL import Image, ImageDraw, ImageFont
import matplotlib.pyplot as plt
from matplotlib import patches

# QR-коды и маркеры
from pyzbar import pyzbar
import cv2.aruco as aruco

# 3D преобразования и математика
import transforms3d as tf3d
from transforms3d.euler import euler2mat, mat2euler
from transforms3d.quaternions import quat2mat, mat2quat

# Логирование и отладка
from rich.console import Console
from rich.logging import RichHandler
from rich.progress import Progress, BarColumn, TextColumn, TimeRemainingColumn
import logging

# Настройка логирования
console = Console()
logging.basicConfig(
    level=logging.INFO,
    format="%(message)s",
    handlers=[RichHandler(rich_tracebacks=True)]
)
logger = logging.getLogger("drone")



def check_environment():
    """Проверка окружения"""
    logger.info("🚀 Дрон готов к работе!")
    logger.info(f"🐍 Python версия: {sys.version}")
    logger.info(f"📦 OpenCV версия: {cv2.__version__}")
    logger.info(f"🔢 NumPy версия: {np.__version__}")

    # Проверка доступности камеры (опционально)
    try:
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            logger.info("📷 Камера доступна")
            cap.release()
        else:
            logger.warning("📷 Камера не доступна")
    except:
        logger.warning("📷 Не удалось проверить камеру")


if __name__ == "__main__":
    check_environment()