#!/usr/bin/env roseus

(ros::roseus "classify")

;; --- 1. グローバル変数定義 ---

;; 現在のセンサ値
(defvar *current-sensor-vector* (instantiate float-vector 32))

;; 学習データの一時保存用: ((:class-name . (vec1 vec2 ...)) ...)
(defvar *raw-training-data* nil)

;; 学習済みモデル: ((:class-name . (mean-vec weight-vec)) ...)
(defvar *trained-models* nil)

;; ゼロ除算防止用の微小値
(defconstant *epsilon* 1e-6)


;; --- 2. 数学ヘルパー関数 ---

;; 要素ごとの掛け算 (Hadamard product)
(defun v-mul (v1 v2)
  (let ((res (instantiate float-vector (length v1))))
    (dotimes (i (length v1))
      (setf (aref res i) (* (aref v1 i) (aref v2 i))))
    res))

;; 重み付きコサイン類似度の計算
;; S = sum(w * x * mu) / (sqrt(sum(w * x^2)) * sqrt(sum(w * mu^2)))
(defun calc-weighted-cosine-similarity (input-vec mean-vec weight-vec)
  (let* ((w-x (v-mul weight-vec input-vec))      ;; w * x
         (w-mu (v-mul weight-vec mean-vec))      ;; w * mu
         (numerator (v. w-x mean-vec))           ;; 分子: sum((w*x) * mu)
         ;; 分母の項1: sqrt(sum(w * x^2)) -> sum((w*x) * x)
         (denom-input (sqrt (abs (v. w-x input-vec))))
         ;; 分母の項2: sqrt(sum(w * mu^2)) -> sum((w*mu) * mu)
         (denom-mean (sqrt (abs (v. w-mu mean-vec)))))

    (if (or (< denom-input *epsilon*) (< denom-mean *epsilon*))
        0.0
        (/ numerator (* denom-input denom-mean)))))

;; 平均ベクトルと重みベクトルを計算する
;; 重み w = |mean| / (variance + epsilon)
(defun calc-stats (vec-list)
  (let* ((len (length (car vec-list)))
         (n (length vec-list))
         (mean-vec (instantiate float-vector len))
         (var-vec (instantiate float-vector len))
         (weight-vec (instantiate float-vector len)))

    ;; 1. 平均の計算
    (dolist (v vec-list)
      (setq mean-vec (v+ mean-vec v)))
    (setq mean-vec (scale (/ 1.0 n) mean-vec))

    ;; 2. 分散の計算 sum((x - mu)^2) / n
    (dolist (v vec-list)
      (let ((diff (v- v mean-vec)))
        (dotimes (i len)
          (setf (aref var-vec i)
                (+ (aref var-vec i) (expt (aref diff i) 2))))))
    (setq var-vec (scale (/ 1.0 n) var-vec))

    ;; 3. 重みの計算 (|mu| / (var + e))
    (dotimes (i len)
      ;; (setf (aref weight-vec i)
      ;;       (/ (abs (aref mean-vec i)) (+ (aref var-vec i) *epsilon*))))
      (let ((mu (aref mean-vec i))
            (var (aref var-vec i)))
        ;; ここを変更しました: abs(mu) -> (expt mu 2)
        (setf (aref weight-vec i)
              (/ (expt mu 2) (+ var *epsilon*)))))

    (list mean-vec weight-vec)))


;; --- 3. データ収集と学習用関数 ---

;; クラス名（シンボル）と現在のセンサ値を保存する関数
;; 使用例: (add-sample :hand-holding-type-a)
(defun add-sample (class-name)
  (let ((entry (assoc class-name *raw-training-data*))
        (vec (copy-object *current-sensor-vector*)))
    (unless entry
      (setq entry (cons class-name nil))
      (push entry *raw-training-data*))
    ;; データの追加 (cdr entry にリストとして追加)
    (nconc entry (list vec))
    (format t "Added sample for ~A. Total: ~A~%" class-name (length (cdr (assoc class-name *raw-training-data*))))))

;; 蓄積されたデータからモデル（平均・重み）を作成する関数
(defun train-models ()
  (setq *trained-models* nil)
  (dolist (entry *raw-training-data*)
    (let* ((class-name (car entry))
           (data-list (cdr entry)))
      (if data-list
          (let ((stats (calc-stats data-list)))
            (push (cons class-name stats) *trained-models*)
            (format t "Trained model for ~A~%" class-name))
          (format t "No data for ~A, skipping.~%" class-name)))))


;; --- 4. ROS Callback ---
(defun sensor-cb (msg)
  (let ((data (send msg :data)))
    ;; Float32MultiArrayのデータはfloat-vectorとして取得可能
    (if (= (length data) 32)
        (setq *current-sensor-vector* data)
        (warn "Received data length is not 32!~%"))))

(ros::subscribe "force_data_diff" std_msgs::Float32MultiArray #'sensor-cb)

;; --- 5. メインループ ---
(defun main-loop ()
  (format t "
=== Interactive Classifier Controller ===
 [1-6]: Add sample for Class 1-6
 [t]  : Train models (Update templates)
 [s]  : Toggle Status display (Inference ON/OFF)
 [r]  : Reset/Clear all data
 [q]  : Quit
-----------------------------------------
")

  (let ((running t)
        (inference-mode nil) ;; 推論結果を表示するかどうかのフラグ
        (key-input nil))

    (while (and (ros::ok) running)
      (ros::spin-once)

      ;; --- A. ノンブロッキングなキー入力処理 ---
      ;; *standard-input* にデータが来ているか 0.0秒だけ確認
      (when (select-stream (list *standard-input*) 0.001)
        (setq key-input (read-char *standard-input*))

        ;; キーに応じた処理の分岐
        (case key-input
          ;; クラスごとのデータ取得
          (#\1 (add-sample :class-1))
          (#\2 (add-sample :class-2))
          (#\3 (add-sample :class-3))
          (#\4 (add-sample :class-4))
          (#\5 (add-sample :class-5))
          (#\6 (add-sample :class-6))

          ;; 学習実行
          (#\t
           (format t "~%--- Training Models ---~%")
           (train-models)
           (format t "--- Training Done ---~%"))

          ;; 推論モード切り替え
          (#\s
           (setq inference-mode (not inference-mode))
           (format t "~%Inference Mode: ~A~%" (if inference-mode "ON" "OFF")))

          ;; データクリア
          (#\r
           (setq *raw-training-data* nil)
           (setq *trained-models* nil)
           (format t "~%All data cleared.~%"))

          ;; 終了
          (#\q (setq running nil))
          ))

      ;; --- B. 推論と表示 (推論モードONのときのみ) ---
      (when (and inference-mode *trained-models*)
        (let ((max-score -1.0)
              (best-class "Unknown"))

          (dolist (model *trained-models*)
            (let* ((class-name (car model))
                   (mean (cadr model))
                   (weight (caddr model))
                   (score (calc-weighted-cosine-similarity *current-sensor-vector* mean weight)))

              (when (> score max-score)
                (setq max-score score)
                (setq best-class class-name))))

          ;; 画面が流れすぎないよう、少し待つか、フォーマットを工夫
          (format t "\rResult: ~10A (Score: ~5,3F)\n" best-class max-score)
          (finish-output *standard-output*)
          ))

      (unix:usleep 100000) ;; CPU負荷軽減 (10ms sleep)
      ))
  (format t "~%Exiting.~%"))

;; 実行
(main-loop)
