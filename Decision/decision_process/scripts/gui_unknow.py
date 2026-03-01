import tkinter as tk
from tkinter import messagebox, simpledialog
import numpy as np
from PIL import Image, ImageTk

class ImageAnnotator:
    def __init__(self, root, image_path):
        self.root = root
        self.root.title("RMUL地图标点 (自适应缩放)")
        
        # 加载原始图片并保持引用
        self.original_image = Image.open(image_path)
        self.orig_width, self.orig_height = self.original_image.size
        
        # 采样滤镜配置 (适应旧版本 Pillow)
        if hasattr(Image, 'Resampling'):
            self.hq_filter = Image.Resampling.BILINEAR
            self.fast_filter = Image.Resampling.NEAREST
        else:
            self.hq_filter = Image.BILINEAR
            self.fast_filter = Image.NEAREST

        # 初始化状态变量
        self.tk_image = None
        self.canvas = tk.Canvas(root, bg="gray", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.annotations_data = []  # 存储逻辑坐标点 (orig_x, orig_y, type, color, text)
        self.points = []            # 存储前 3 个点击的原始坐标点
        self.real_points = []       # 存储对应的现实坐标
        self.transformation_matrix = None

        # 缩放平移状态
        self.zoom_scale = 1.0
        self.pan_x = 0
        self.pan_y = 0
        self.last_mouse_x = 0
        self.last_mouse_y = 0

        # 渲染与缓存状态
        self.render_timer = None
        self.hq_render_timer = None
        self.cache_scale = -1.0    
        
        # 绑定事件
        self.canvas.bind("<Button-1>", self.handle_click)
        self.canvas.bind("<Button-3>", self.remove_annotation)
        self.canvas.bind("<Configure>", self.on_resize)
        
        # 滚轮缩放
        self.canvas.bind("<Button-4>", self.zoom_in)
        self.canvas.bind("<Button-5>", self.zoom_out)
        
        # 中键平移
        self.canvas.bind("<Button-2>", self.start_pan)
        self.canvas.bind("<B2-Motion>", self.do_pan)
        
        self.queue_render(100)

    def queue_render(self, ms=30, high_quality=False):
        if self.render_timer is not None:
            self.root.after_cancel(self.render_timer)
        
        if high_quality:
            self.render(use_hq=True)
        else:
            self.render_timer = self.root.after(ms, lambda: self.render(use_hq=False))
            if self.hq_render_timer is not None:
                self.root.after_cancel(self.hq_render_timer)
            self.hq_render_timer = self.root.after(200, lambda: self.render(use_hq=True))

    def zoom_in(self, event):
        self.zoom_scale *= 1.25
        self.queue_render(10)

    def zoom_out(self, event):
        self.zoom_scale /= 1.25
        if self.zoom_scale < 0.05: self.zoom_scale = 0.05
        self.queue_render(10)

    def start_pan(self, event):
        self.last_mouse_x = event.x
        self.last_mouse_y = event.y

    def do_pan(self, event):
        dx = event.x - self.last_mouse_x
        dy = event.y - self.last_mouse_y
        self.pan_x += dx
        self.pan_y += dy
        self.last_mouse_x = event.x
        self.last_mouse_y = event.y
        self.queue_render(5)

    def on_resize(self, event):
        self.queue_render(50)

    def render(self, use_hq=False):
        self.render_timer = None
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()
        if canvas_width <= 1 or canvas_height <= 1: return

        fit_ratio = min(canvas_width / self.orig_width, canvas_height / self.orig_height)
        self.current_ratio = fit_ratio * self.zoom_scale
        
        virtual_w = self.orig_width * self.current_ratio
        virtual_h = self.orig_height * self.current_ratio
        v_draw_x = (canvas_width - virtual_w) / 2 + self.pan_x
        v_draw_y = (canvas_height - virtual_h) / 2 + self.pan_y
        
        sx1, sy1 = max(0, v_draw_x), max(0, v_draw_y)
        sx2, sy2 = min(canvas_width, v_draw_x + virtual_w), min(canvas_height, v_draw_y + virtual_h)
        if sx1 >= sx2 or sy1 >= sy2:
            self.canvas.delete("all")
            return

        ox1 = (sx1 - v_draw_x) / self.current_ratio
        oy1 = (sy1 - v_draw_y) / self.current_ratio
        ox2 = (sx2 - v_draw_x) / self.current_ratio
        oy2 = (sy2 - v_draw_y) / self.current_ratio
        display_w, display_h = int(sx2 - sx1), int(sy2 - sy1)
        
        state_key = (self.current_ratio, self.pan_x, self.pan_y, use_hq, canvas_width, canvas_height)
        if not hasattr(self, '_last_state') or self._last_state != state_key:
            filt = self.hq_filter if use_hq else self.fast_filter
            crop_box = (max(0, ox1), max(0, oy1), min(self.orig_width, ox2), min(self.orig_height, oy2))
            cropped = self.original_image.crop(crop_box)
            resized = cropped.resize((display_w, display_h), filt)
            self.tk_image = ImageTk.PhotoImage(resized)
            self._last_state = state_key
            self._last_render_fast = not use_hq
            self.screen_pos = (sx1, sy1)

        self.canvas.delete("all")
        self.canvas.create_image(self.screen_pos[0], self.screen_pos[1], anchor='nw', image=self.tk_image)
        self.draw_pos_x, self.draw_pos_y = v_draw_x, v_draw_y
        self.redraw_annotations()

    def redraw_annotations(self):
        for data in self.annotations_data:
            orig_x, orig_y, p_type, color, text = data
            screen_x = orig_x * self.current_ratio + self.draw_pos_x
            screen_y = orig_y * self.current_ratio + self.draw_pos_y
            if -50 < screen_x < self.canvas.winfo_width() + 50 and \
               -50 < screen_y < self.canvas.winfo_height() + 50:
                self.canvas.create_oval(screen_x - 5, screen_y - 5, screen_x + 5, screen_y + 5, fill=color, outline="black")
                self.canvas.create_text(screen_x, screen_y - 25, text=text, anchor='n', fill=color)

    def handle_click(self, event):
        if not hasattr(self, 'current_ratio'): return
        orig_x = (event.x - self.draw_pos_x) / self.current_ratio
        orig_y = (event.y - self.draw_pos_y) / self.current_ratio

        if 0 <= orig_x <= self.orig_width and 0 <= orig_y <= self.orig_height:
            if len(self.points) < 3:
                self.points.append((orig_x, orig_y))
                self.annotations_data.append([orig_x, orig_y, "calib", "red", f"P{len(self.points)}"])
                self.queue_render()

                if len(self.points) == 3:
                    for i in range(3):
                        coords = simpledialog.askstring(f"输入点 P{i+1} 的现实坐标", 
                                                       f"请输入第 P{i+1} 个点的世界物理坐标 (x,y)\n注意：y轴方向由你定义的三个点决定", parent=self.root)
                        if coords:
                            try:
                                rx, ry = map(float, coords.split(','))
                                self.real_points.append((rx, ry))
                            except:
                                messagebox.showerror("错误", "坐标格式不对，请使用 1.2, 3.4 这种格式")
                                self.reset_calibration()
                                return
                        else:
                            self.reset_calibration()
                            return
                    self.calculate_transformation_matrix()
            else:
                if self.transformation_matrix is not None:
                    res = self.transform_coordinates(orig_x, orig_y)
                    self.annotations_data.append([orig_x, orig_y, "point", "blue", f"({res[0]:.3f}, {res[1]:.3f})"])
                    self.queue_render()

    def reset_calibration(self):
        self.points = []
        self.real_points = []
        self.transformation_matrix = None
        self.annotations_data = [d for d in self.annotations_data if d[2] != "calib"]
        self.queue_render()

    def calculate_transformation_matrix(self):
        # 使用 3 个点求解仿射变换的 6 个参数：
        # real_x = a*x + b*y + c
        # real_y = d*x + e*y + f
        A = []
        bx = []
        by = []
        for (x, y), (real_x, real_y) in zip(self.points, self.real_points):
            A.append([x, y, 1])
            bx.append(real_x)
            by.append(real_y)
        
        A = np.array(A)
        try:
            # 独立求解 X 和 Y 的变换参数
            res_x = np.linalg.solve(A, bx)
            res_y = np.linalg.solve(A, by)
            self.transformation_matrix = np.array([res_x, res_y])
            messagebox.showinfo("成功", "3点校准完成！现在坐标映射已固定。")
        except np.linalg.LinAlgError:
            messagebox.showerror("错误", "选取的三个点不能在同一直线上！")
            self.reset_calibration()

    def transform_coordinates(self, x, y):
        point = np.array([x, y, 1])
        res = np.dot(self.transformation_matrix, point)
        return res

    def remove_annotation(self, event):
        if not hasattr(self, 'current_ratio'): return
        orig_x = (event.x - self.draw_pos_x) / self.current_ratio
        orig_y = (event.y - self.draw_pos_y) / self.current_ratio
        
        # 寻找最近的点删除
        threshold = 10 / self.current_ratio # 屏幕上10像素的感应距离
        for data in self.annotations_data[:]:
            dx = data[0] - orig_x
            dy = data[1] - orig_y
            if (dx*dx + dy*dy)**0.5 < threshold:
                self.annotations_data.remove(data)
                # 如果删除了校准点，重置校准状态
                if data[2] == "calib":
                    self.points = []
                    self.real_points = []
                    self.transformation_matrix = None
                break
        self.queue_render(10)

if __name__ == "__main__":
    import os
    root = tk.Tk()
    root.geometry("1000x800") # 默认一个较大的窗口
    script_dir = os.path.dirname(os.path.abspath(__file__))
    image_path = os.path.join(script_dir, "26RMUC.png")
        
    app = ImageAnnotator(root, image_path)
    root.mainloop()