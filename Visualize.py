import cv2
import matplotlib.pyplot as plt
import numpy as np

# 1. READ IMAGE
# Görüntüyü yükle
image_path = r"C:\Users\HP\Desktop\aynakaynak-main\pipe_image.png"
original_img = cv2.imread(image_path)

if original_img is None:
    print("Hata: Görüntü yüklenemedi. pipe_image.png dosyasının varlığından emin olun.")
    exit()

# Helper function to display using Matplotlib
def show_figure(img, title, is_gray=False):
    plt.figure(figsize=(10, 6))
    if is_gray:
        plt.imshow(img, cmap='gray')
    else:
        # BGR -> RGB conversion for Matplotlib
        rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        plt.imshow(rgb_img)
    plt.title(title)
    plt.axis("off")

# STEP 1: Original Image
# Adım 1: Orijinal Görüntü
show_figure(original_img, "Adim 1: Orijinal Goruntu")

# STEP 1.5: ROI (Region of Interest) Filtering
# Lazer çizgisi genellikle yataydır ve en parlak alandır.
# Görüntünün y ekseni boyunca parlaklık toplamını alarak lazerin nerede olduğunu bulalım.
gray_full = cv2.cvtColor(original_img, cv2.COLOR_BGR2GRAY)
row_sums = np.sum(gray_full, axis=1) # Satır toplamları
max_row_index = np.argmax(row_sums) # En parlak satırın indexi

# ROI Yüksekliği (Lazerin kalınlığına göre ayarlanabilir, örn: +/- 60 piksel)
roi_height = 60
h, w = original_img.shape[:2]

y_min = max(0, max_row_index - roi_height)
y_max = min(h, max_row_index + roi_height)

# Maske oluştur ve maske dışını siyah yap
roi_mask = np.zeros_like(gray_full)
roi_mask[y_min:y_max, :] = 255

# Görüntüyü maskele (Sadece lazer bandı kalsın)
masked_img = cv2.bitwise_and(original_img, original_img, mask=roi_mask)
show_figure(masked_img, "Adim 1.5: ROI Mod (Lazer Bandi)", is_gray=False)


# STEP 2: Grayscale (Maskelenmiş görüntü üzerinden)
# Adım 2: Gri Tonlama
gray = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY)
show_figure(gray, "Adim 2: Gri Tonlama (Grayscale)", is_gray=True)

# STEP 3: Blur
# Adım 3: Bulanıklaştırma (Gürültü azaltma)
# 5x5 GaussianBlur kullanıyoruz
blurred = cv2.GaussianBlur(gray, (5, 5), 0)
show_figure(blurred, "Adim 3: Bulaniklastirma (Blur)", is_gray=True)

# STEP 4: Threshold
# Adım 4: Eşikleme (Threshold)
# Maskeleme sonrası arka plan zaten siyah olduğu için standart threshold daha iyi çalışır.
# 100-150 arası bir değer genel olarak iyidir.
_, binary = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)
show_figure(binary, "Adim 4: Esikleme (Threshold)", is_gray=True)

# STEP 5: Morphological Operations
# Adım 5: Morfolojik İşlemler (Clean & Connect)

# 1. MORPH_OPEN: Çizgi kenarlarındaki pürüzleri (saçakları) temizler.
# GÜNCELLENDİ: Kaldırıldı. İnce detayları siliyordu.
# kernel_open = np.ones((2, 2), np.uint8)
# morph = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel_open)
morph = binary # Open adımını atla

# 2. MORPH_CLOSE: Sadece DIKEY boşlukları kapatmak için dikey kernel kullanalım.
# Yatay (horizontal) boşlukları kapatmasın ki boruları tespit edebilelim.
kernel_close_vertical = np.ones((7, 1), np.uint8) 
morph = cv2.morphologyEx(morph, cv2.MORPH_CLOSE, kernel_close_vertical)

# Ekstra Temizlik: Çok küçük gürültüleri atalım (Contours ile)
cnts_noise, _ = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
mask_clean = np.zeros_like(morph)
for c in cnts_noise:
    # GÜNCELLENDİ: Eşik 10 -> 5 düşürüldü. En sağdaki minik parçayı kurtarmak için.
    if cv2.contourArea(c) > 5: 
        cv2.drawContours(mask_clean, [c], -1, 255, -1)

morph = mask_clean # Temizlenmiş hali kullan
show_figure(morph, "Adim 5: Morfolojik Islemler (Clean Only)", is_gray=True)

# STEP 6: Contour Detection
# Adım 6: Kontur Bulma
# Beyaz segmentlerin (lazer parçalarının) sınırlarını buluyoruz.
cnts, _ = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# STEP 7: Gap Detection (Pipes)
# Adım 7: Boşluk Tespiti (Borular)
# Konturları soldan sağa sırala (x eksenine göre)
if len(cnts) > 0:
    # bounding rect'in x değerine göre sırala
    cnts = sorted(cnts, key=lambda c: cv2.boundingRect(c)[0])
    
    # Görselleştirme için kopya al
    final_img = original_img.copy()
    
    # Her ardışık iki segment arasındaki boşluğa bak
    for i in range(len(cnts) - 1):
        c1 = cnts[i]
        c2 = cnts[i+1]
        
        # Segmentlerin kutularını al (x, y, genişlik, yükseklik)
        x1, y1, w1, h1 = cv2.boundingRect(c1)
        x2, y2, w2, h2 = cv2.boundingRect(c2)
        
        # Soldaki segmentin bitişi (sağ ucu)
        c1_end_x = x1 + w1
        # Sağdaki segmentin başlangıcı (sol ucu)
        c2_start_x = x2
        
        # Aradaki boşluk (Gap)
        gap = c2_start_x - c1_end_x
        
        # Eğer boşluk anlamlı bir büyüklükteyse (GÜNCELLENDİ: 40 piksel)
        if gap > 50:
            # Boşluğun merkezi
            center_x = int(c1_end_x + gap / 2)
            # Yüksekliğin merkezi (iki segmentin ortalaması)
            center_y = int((y1 + h1/2 + y2 + h2/2) / 2)
            
            radius = int(gap / 2)
            
            # Çember çiz (Sarı renk: BGR -> 0, 255, 255)
            cv2.circle(final_img, (center_x, center_y), radius, (0, 255, 255), 2)
            
            # Yazı yaz (Gap değeri)
            cv2.putText(final_img, f"{gap} px", (center_x - 20, center_y - radius - 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # Çap çizgisini çiz (Opsiyonel)
            cv2.line(final_img, (c1_end_x, center_y), (c2_start_x, center_y), (255, 0, 0), 1)

    show_figure(final_img, "Adim 7: Sonuc (Tespit Edilen Bosluklar)")

# Tüm grafikleri göster
plt.show()
