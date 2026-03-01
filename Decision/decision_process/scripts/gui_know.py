import os
import tkinter as tk
from tkinter import messagebox, simpledialog
from PIL import Image, ImageTk


def to_RMUC(x, y, w, h):
    # 原点在左上角，向右x增大(0→28)，向下y增大(0→15)
    return f"({x / w * 28:.3f},{y / h * 15:.3f})"

def to_RMUL(x, y, w, h):
    # rmul2026.pgm: 253x173 像素，RMUL 地图 12m x 8m
    # 原点在左上角，向右x增大(0→12)，向下y增大(0→8)
    return f"({x / w * 12:.3f},{y / h * 8:.3f})"

def to_QINGQING(x, y, w, h):
    return f"({x / w * 21.75:.3f},{9.85 - y / h * 9.85:.3f})"


class ImageAnnotator:
    def __init__(self, root, image_path):
        self.root = root
        self.root.title("地图标点")
        
        # Load the original image using PIL
        self.image_path = image_path
        self.image_original = Image.open(image_path)
        self.orig_w, self.orig_h = self.image_original.size
        
        # Create a canvas that fills the window
        self.canvas = tk.Canvas(root, width=self.orig_w, height=self.orig_h, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

        # Display the image
        self.tk_image = ImageTk.PhotoImage(self.image_original)
        self.image_id = self.canvas.create_image(0, 0, anchor='nw', image=self.tk_image)
        
        self.annotations = []
        self.current_w = self.orig_w
        self.current_h = self.orig_h

        self.canvas.bind("<Button-1>", self.add_annotation)
        self.canvas.bind("<Button-3>", self.remove_annotation)
        self.canvas.bind("<Configure>", self.on_resize)

    def on_resize(self, event):
        # Resize image to fit the new canvas size
        new_w = event.width
        new_h = event.height
        
        if new_w < 10 or new_h < 10:  # Avoid resizing to invisible size
            return
            
        if new_w == self.current_w and new_h == self.current_h:
            return

        # Scale factor
        x_scale = new_w / self.current_w
        y_scale = new_h / self.current_h

        # Update image
        resized_img = self.image_original.resize((new_w, new_h), Image.LANCZOS)
        self.tk_image = ImageTk.PhotoImage(resized_img)
        self.canvas.itemconfig(self.image_id, image=self.tk_image)

        # Scale all existing annotations (circles and labels)
        self.canvas.scale("all", 0, 0, x_scale, y_scale)
        
        # Update current dimensions
        self.current_w = new_w
        self.current_h = new_h

    def add_annotation(self, event):
        x, y = event.x, event.y
        circle = self.canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill="red", outline="black")

        # Create a label to show the coordinates
        coord_text = to_QINGQING(x, y, self.current_w, self.current_h)
        label = self.canvas.create_text(x, y - 25, text=coord_text, anchor='n', fill='red')

        # Bind a click event to the label to update its position
        self.canvas.tag_bind(label, "<Button-1>", lambda e, l=label, c=circle: self.on_label_click(e, l, c))

        self.annotations.append((circle, label))

    def on_label_click(self, event, label, circle):
        # The text is already formatted as "(x,y)", we can extract values if needed, 
        # but for updating we might want to prompt for real-world coords or pixel coords.
        # The original code asked for pixel coords in simpledialog.
        
        # Get current canvas coordinates for the circle
        coords = self.canvas.coords(circle)
        curr_x = (coords[0] + coords[2]) / 2
        curr_y = (coords[1] + coords[3]) / 2

        # Prompt for new real-world coordinates or pixels? 
        # The original code seemed to expect pixels in the prompt: "initialvalue=to_RMUL(x, y)" 
        # but then used "new_x, new_y" directly in Canvas.coords.
        # This implies it expects pixel coordinates.
        
        new_coords = simpledialog.askstring("Update Coordinates", "Enter new pixel coordinates (x,y):",
                                            parent=self.root, initialvalue=f"{int(curr_x)},{int(curr_y)}")

        if new_coords:
            try:
                new_x, new_y = map(float, new_coords.split(','))
                # Update the circle and label positions
                self.canvas.coords(circle, new_x - 5, new_y - 5, new_x + 5, new_y + 5)
                self.canvas.coords(label, new_x, new_y - 25)
                self.canvas.itemconfig(label, text=to_QINGQING(new_x, new_y, self.current_w, self.current_h))
            except ValueError:
                pass

    def remove_annotation(self, event):
        x, y = event.x, event.y
        items = self.canvas.find_overlapping(x - 5, y - 5, x + 5, y + 5)
        for item in items:
            for circle, label in self.annotations:
                if item == circle or item == label:
                    self.canvas.delete(circle)
                    self.canvas.delete(label)
                    self.annotations.remove((circle, label))
                    break


if __name__ == "__main__":
    root = tk.Tk()
    script_dir = os.path.dirname(os.path.abspath(__file__))
    image_path = os.path.join(script_dir, "qingqing2026.pgm")
    app = ImageAnnotator(root, image_path)
    root.mainloop()