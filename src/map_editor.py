import os
import yaml
from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, 
    QFileDialog, QSlider, QComboBox, QMessageBox, QScrollArea, QWidget
)
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen, QColor
from PyQt5.QtCore import Qt, QPoint

class MapEditorDialog(QDialog):
    def __init__(self, parent=None, initial_map_yaml=None):
        super().__init__(parent)
        self.setWindowTitle("Trình Chỉnh Sửa Map (Map Editor)")
        self.resize(1000, 700)
        
        self.yaml_path = None
        self.image_path = None
        self.qimage = None
        
        # Drawing state
        self.drawing = False
        self.last_point = QPoint()
        self.brush_size = 5
        self.draw_color = QColor(0, 0, 0)  # Default: Obstacle (Black)
        self.scale_factor = 1.0
        
        self.init_ui()
        if initial_map_yaml and os.path.exists(initial_map_yaml):
            self.load_map(initial_map_yaml)
            
    def init_ui(self):
        main_layout = QVBoxLayout(self)
        
        # Top toolbar
        toolbar = QHBoxLayout()
        
        self.btn_load = QPushButton("Mở Map (.yaml)")
        self.btn_save = QPushButton("Lưu Map")
        self.btn_load.clicked.connect(self.on_load_clicked)
        self.btn_save.clicked.connect(self.save_map)
        
        self.cmb_tool = QComboBox()
        self.cmb_tool.addItems([
            "Vẽ vật cản (Đen/Obstacle)", 
            "Xóa vật cản (Trắng/Free Space)", 
            "Vùng không rõ (Xám/Unknown)"
        ])
        self.cmb_tool.currentIndexChanged.connect(self.on_tool_changed)
        
        self.slider_brush = QSlider(Qt.Horizontal)
        self.slider_brush.setMinimum(1)
        self.slider_brush.setMaximum(50)
        self.slider_brush.setValue(self.brush_size)
        self.slider_brush.valueChanged.connect(self.on_brush_changed)
        
        self.lbl_info = QLabel("Map: Chưa tải")
        
        toolbar.addWidget(self.btn_load)
        toolbar.addWidget(self.btn_save)
        toolbar.addWidget(QLabel("Công cụ:"))
        toolbar.addWidget(self.cmb_tool)
        self.btn_zoom_in = QPushButton("Zoom In (+)")
        self.btn_zoom_out = QPushButton("Zoom Out (-)")
        self.btn_zoom_in.clicked.connect(self.zoom_in)
        self.btn_zoom_out.clicked.connect(self.zoom_out)

        toolbar.addWidget(QLabel("Cỡ cọ:"))
        toolbar.addWidget(self.slider_brush)
        toolbar.addWidget(self.btn_zoom_out)
        toolbar.addWidget(self.btn_zoom_in)
        toolbar.addWidget(self.lbl_info)
        toolbar.addStretch()
        
        main_layout.addLayout(toolbar)
        
        # Map view area
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.lbl_map = QLabel("Chọn file map để chỉnh sửa.")
        self.lbl_map.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        
        # Enable mouse tracking for drawing
        self.lbl_map.mousePressEvent = self.mousePressEvent_map
        self.lbl_map.mouseMoveEvent = self.mouseMoveEvent_map
        self.lbl_map.mouseReleaseEvent = self.mouseReleaseEvent_map
        
        self.scroll_area.setWidget(self.lbl_map)
        main_layout.addWidget(self.scroll_area)
        
    def on_tool_changed(self, idx):
        if idx == 0:
            self.draw_color = QColor(0, 0, 0) # Black (Obstacle)
        elif idx == 1:
            self.draw_color = QColor(254, 254, 254) # White (Free space)
        else:
            self.draw_color = QColor(205, 205, 205) # Gray (Unknown)
            
    def on_brush_changed(self, val):
        self.brush_size = val
        
    def on_load_clicked(self):
        start_dir = os.path.dirname(self.yaml_path) if self.yaml_path else os.path.expanduser("~")
        file_path, _ = QFileDialog.getOpenFileName(self, "Chọn file map YAML", start_dir, "YAML files (*.yaml *.yml)")
        if file_path:
            self.load_map(file_path)
            
    def load_map(self, yaml_path):
        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                
            img_filename = data.get("image", None)
            if not img_filename:
                raise ValueError("Không tìm thấy trường 'image' trong file YAML.")
                
            # Resolve image path relative to yaml file
            yaml_dir = os.path.dirname(yaml_path)
            if not os.path.isabs(img_filename):
                self.image_path = os.path.join(yaml_dir, img_filename)
            else:
                self.image_path = img_filename
                
            if not os.path.exists(self.image_path):
                raise FileNotFoundError(f"Không tìm thấy file ảnh map: {self.image_path}")
                
            # Load QImage
            self.qimage = QImage(self.image_path)
            if self.qimage.isNull():
                raise ValueError("Lỗi khi đọc file ảnh (PGM).")
                
            # Ensure format is RGB32 for drawing
            if self.qimage.format() != QImage.Format_RGB32:
                self.qimage = self.qimage.convertToFormat(QImage.Format_RGB32)
                
            self.yaml_path = yaml_path
            self.lbl_info.setText(f"Map: {os.path.basename(yaml_path)}")
            self.update_display()
            
        except Exception as e:
            QMessageBox.critical(self, "Lỗi Tải Map", str(e))
            
    def zoom_in(self):
        self.scale_factor *= 1.25
        self.update_display()

    def zoom_out(self):
        self.scale_factor *= 0.8
        self.update_display()

    def update_display(self):
        if self.qimage and not self.qimage.isNull():
            pixmap = QPixmap.fromImage(self.qimage)
            if self.scale_factor != 1.0:
                new_width = int(pixmap.width() * self.scale_factor)
                new_height = int(pixmap.height() * self.scale_factor)
                pixmap = pixmap.scaled(new_width, new_height, Qt.KeepAspectRatio, Qt.FastTransformation)
            self.lbl_map.setPixmap(pixmap)
            self.lbl_map.resize(pixmap.size())
            
    def save_map(self):
        if not self.qimage or self.qimage.isNull():
            QMessageBox.warning(self, "Cảnh báo", "Không có map nào đang mở để lưu.")
            return
            
        if self.image_path:
            # Convert back to Grayscale/Indexed before saving PGM
            save_img = self.qimage.convertToFormat(QImage.Format_Grayscale8)
            success = save_img.save(self.image_path)
            if success:
                QMessageBox.information(self, "Thành công", f"Đã lưu đè thành công lên:\n{self.image_path}")
            else:
                QMessageBox.critical(self, "Lỗi", "Không thể ghi đè lên file ảnh.")

    def map_to_image_pos(self, pos):
        if self.scale_factor == 1.0:
            return pos
        return QPoint(int(pos.x() / self.scale_factor), int(pos.y() / self.scale_factor))

    def mousePressEvent_map(self, event):
        if event.button() == Qt.LeftButton and self.qimage:
            self.drawing = True
            self.last_point = self.map_to_image_pos(event.pos())
            self.draw_on_map(self.map_to_image_pos(event.pos()))

    def mouseMoveEvent_map(self, event):
        if (event.buttons() & Qt.LeftButton) and self.drawing and self.qimage:
            self.draw_on_map(self.map_to_image_pos(event.pos()))

    def mouseReleaseEvent_map(self, event):
        if event.button() == Qt.LeftButton:
            self.drawing = False

    def draw_on_map(self, current_point):
        # Prevent drawing outside bounds
        if current_point.x() < 0 or current_point.y() < 0 or current_point.x() >= self.qimage.width() or current_point.y() >= self.qimage.height():
            return
            
        painter = QPainter(self.qimage)
        pen = QPen(self.draw_color, self.brush_size, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
        painter.setPen(pen)
        painter.drawLine(self.last_point, current_point)
        painter.end()
        self.last_point = current_point
        self.update_display()
