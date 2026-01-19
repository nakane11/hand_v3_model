#!/usr/bin/env roseus

(ros::roseus "classify")

;; --- 1. グローバル変数・定数定義 ---

;; センサの最大値 (12bit = 4095)
(defconstant *sensor-max-val* 4095.0)

;; 正規化後の接触判定閾値 (0.0~1.0スケール)
(defconstant *contact-threshold* 0.3)

;; 類似度判定の閾値
(defconstant *similarity-threshold* 0.55)

;; ゼロ除算防止用の微小値
(defconstant *epsilon* 1e-6)

;; --- パラメータ調整用定数 ---

;; 分散のベースライン (ノイズフロア)
;; これを大きくすると、重みの最大値が下がり、判定がマイルドになります。
;; 推奨: 0.001 ~ 0.01
(defconstant *base-variance* 0.005)

;; 重みの最大値リミッター
(defconstant *max-weight-limit* 200.0)

;; --- 変数定義 ---

;; 現在のセンサ値 (常に0.0~1.0に正規化される)
(defvar *current-sensor-vector* (instantiate float-vector 32))

;; ★保存対象: 生の学習データ ((:class-name . (vec1 vec2 ...)) ...)
(defvar *raw-training-data* nil)

;; 学習済みモデル (毎回計算する)
(defvar *trained-models* nil)

(defvar *pub-msg* (instance std_msgs::Int8 :init))
(defvar *score-msg* (instance std_msgs::Float32 :init))

;; --- 2. 数学ヘルパー関数 ---

(defun v-mul (v1 v2)
  (let ((res (instantiate float-vector (length v1))))
    (dotimes (i (length v1))
      (setf (aref res i) (* (aref v1 i) (aref v2 i))))
    res))

(defun calc-weighted-cosine-similarity (input-vec mean-vec weight-vec)
  (let* ((w-x (v-mul weight-vec input-vec))
         (w-mu (v-mul weight-vec mean-vec))
         (numerator (v. w-x mean-vec))
         (denom-input (sqrt (abs (v. w-x input-vec))))
         (denom-mean (sqrt (abs (v. w-mu mean-vec)))))

    (if (or (< denom-input *epsilon*) (< denom-mean *epsilon*))
        0.0
        (/ numerator (* denom-input denom-mean)))))

;; 平均ベクトルと重みベクトルを計算する
;; 最新ロジック: 平均値を分子に使わず、分散の逆数だけで計算
(defun calc-stats (vec-list)
  (let* ((len (length (car vec-list)))
         (n (length vec-list))
         (mean-vec (instantiate float-vector len))
         (var-vec (instantiate float-vector len))
         (weight-vec (instantiate float-vector len)))

    ;; 1. 平均
    (dolist (v vec-list)
      (setq mean-vec (v+ mean-vec v)))
    (setq mean-vec (scale (/ 1.0 n) mean-vec))

    ;; 2. 分散
    (dolist (v vec-list)
      (let ((diff (v- v mean-vec)))
        (dotimes (i len)
          (setf (aref var-vec i)
                (+ (aref var-vec i) (expt (aref diff i) 2))))))
    (setq var-vec (scale (/ 1.0 n) var-vec))

    ;; 3. 重み計算 (分散の逆数アプローチ)
    (dotimes (i len)
      (let ((var (aref var-vec i)))
        ;; 分子に平均値を使わないことで、値が0で安定している箇所も重視する
        (let ((raw-weight (/ 1.0 (+ var *base-variance*))))
          (setf (aref weight-vec i)
                (min raw-weight *max-weight-limit*)))))

    (list mean-vec weight-vec)))


;; --- 3. データ収集と学習用関数 ---

(defun add-sample (class-name)
  (let ((entry (assoc class-name *raw-training-data*))
        (vec (copy-object *current-sensor-vector*)))
    (unless entry
      (setq entry (cons class-name nil))
      (push entry *raw-training-data*))
    (nconc entry (list vec))
    (format t "Added sample for ~A. Total: ~A~%" class-name (length (cdr (assoc class-name *raw-training-data*))))))

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


;; --- 4. ファイル入出力 (RAWデータ保存版) ---

(defun save-raw-data (filename)
  (if *raw-training-data*
      (progn
        ;; 生データを保存する
        (dump-loadable-structure filename *raw-training-data*)
        (format t "[System] Saved RAW DATA to ~A~%" filename))
      (warn "[System] No raw data to save!~%")))

(defun load-raw-data (filename)
  (if (probe-file filename)
      (progn
        ;; 生データを読み込む (変数 *raw-training-data* が復元される)
        (load filename)
        (format t "[System] Loaded RAW DATA from ~A.~%" filename)
        (format t "[System] Please press 't' to re-train with current parameters.~%")) ;; 重要
      (warn "[System] File ~A not found!~%" filename)))


;; --- 5. ROS Callback ---
(defun get-class-id (class-sym)
  (case class-sym
    (:class-1 1)
    (:class-2 2)
    (:class-3 3)
    (:class-4 4)
    (:class-5 5)
    (:class-6 6)
    (t -1))) ;; 想定外のクラス名

(defun sensor-cb (msg)
  (let ((data (send msg :data)))
    (if (= (length data) 32)
        (setq *current-sensor-vector* (scale (/ 1.0 *sensor-max-val*) data))
        (warn "Received data length is not 32!~%"))))

(ros::advertise "hand_class" std_msgs::Int8 1)
(ros::advertise "classification_score" std_msgs::Float32 1)
(ros::subscribe "force_data_diff" std_msgs::Float32MultiArray #'sensor-cb)


;; --- 6. メインループ ---

(defun main-loop ()
  (format t "
=== Interactive Classifier (Raw Data Mode) ===
 [1-6]: Add sample
 [t]  : Train models (Recalculate weights)
 [s]  : Toggle Inference
 [r]  : Reset ALL data
 [w]  : Save RAW DATA (training_data.l)
 [l]  : Load RAW DATA (training_data.l)
 [q]  : Quit
----------------------------------------------
")

  (let ((running t)
        (inference-mode nil)
        (key-input nil))

    (while (and (ros::ok) running)
      (ros::spin-once)

      ;; --- A. キー入力 ---
      (when (select-stream (list *standard-input*) 0.001)
        (setq key-input (read-char *standard-input*))
        (case key-input
          (#\1 (add-sample :class-1))
          (#\2 (add-sample :class-2))
          (#\3 (add-sample :class-3))
          (#\4 (add-sample :class-4))
          (#\5 (add-sample :class-5))
          (#\6 (add-sample :class-6))

          (#\t
           (format t "~%--- Training Models ---~%")
           (train-models) ;; 現在の raw data と パラメータ で再計算
           (format t "--- Training Done ---~%"))

          (#\s
           (setq inference-mode (not inference-mode))
           (format t "~%Inference Mode: ~A~%" (if inference-mode "ON" "OFF")))

          (#\r
           (setq *raw-training-data* nil)
           (setq *trained-models* nil)
           (format t "~%All data cleared.~%"))

          ;; 保存と読み込み
          (#\w (save-raw-data "training_data.l"))
          (#\l (load-raw-data "training_data.l"))

          (#\q (setq running nil))
          ))

      ;; --- B. 推論 ---
      (when (and inference-mode *trained-models*)
        (let ((input-norm (norm *current-sensor-vector*))
              (publish-val 0)
              (publish-score 0.0))
          (cond
           ((< input-norm *contact-threshold*)
            (setq publish-val 0)
            (setq publish-score 0.0)
            (format t "Status: No Contact (Level: ~5,3F)~%" input-norm))

            (t
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

               (setq publish-score max-score)
               (if (> max-score *similarity-threshold*)
                   (progn
                     (setq publish-val (get-class-id best-class))
                     (format t "Result: ~10A (Score: ~5,3F, Level: ~5,3F)~%" best-class max-score input-norm))
                 (progn
                   (setq publish-val -1)
                   (format t "Result: UNKNOWN    (Score: ~5,3F, Level: ~5,3F)~%" max-score input-norm)))
               )))
          (send *pub-msg* :data publish-val)
          (ros::publish "hand_class" *pub-msg*)
          (send *score-msg* :data publish-score)
          (ros::publish "classification_score" *score-msg*)

        (finish-output *standard-output*)))

      (ros::duration-sleep 0.1)))
  (format t "~%Exiting.~%"))

(main-loop)
