#!/usr/bin/env roseus

(ros::roseus "classify")

;; --- 1. グローバル変数・定数定義 ---

;; センサの最大値 (12bit)
(defconstant *sensor-max-val* 4095.0)
;; 正規化後の接触判定閾値 (0.0~1.0)
(defconstant *contact-threshold* 0.3)
;; 類似度判定の閾値
(defconstant *similarity-threshold* 0.55)
;; ゼロ除算防止用の微小値
(defconstant *epsilon* 1e-6)
;; 分散のベースライン
;; これを大きくすると、重みの最大値が下がり、判定がマイルドになります。
;; 推奨: 0.001 ~ 0.01
(defconstant *base-variance* 0.005)
;; 重みの最大値リミッタ
(defconstant *max-weight-limit* 200.0)

;; --- 変数定義 ---
;; 現在のセンサ値 (0.0~1.0に正規化)
(defvar *current-sensor-vector* (instantiate float-vector 32))
;; 保存対象: 生の学習データ
(defvar *raw-training-data* nil)
;; 学習済みモデル (毎回計算する)
(defvar *trained-models* nil)
(defvar *monitoring-target* nil)

(defvar *pub-msg* (instance std_msgs::Int8 :init))
(defvar *score-msg* (instance std_msgs::Float32 :init))

;; --- 2. 数学ヘルパー関数 ---
(defun v-mul (v1 v2)
  (let ((res (instantiate float-vector (length v1))))
    (dotimes (i (length v1))
      (setf (aref res i) (* (aref v1 i) (aref v2 i))))
    res))

(defun calc-stats (vec-list)
  (let* ((len (length (car vec-list)))
         (n (length vec-list))
         (mean-vec (instantiate float-vector len))
         (var-vec (instantiate float-vector len))
         (weight-vec (instantiate float-vector len)))
    ;; 平均値計算
    (dolist (v vec-list)
      (setq mean-vec (v+ mean-vec v)))
    (setq mean-vec (scale (/ 1.0 n) mean-vec))
    ;; 分散計算
    (dolist (v vec-list)
      (let ((diff (v- v mean-vec)))
        (dotimes (i len)
          (setf (aref var-vec i)
                (+ (aref var-vec i) (expt (aref diff i) 2))))))
    (setq var-vec (scale (/ 1.0 n) var-vec))
    ;; 重み計算
    (dotimes (i len)
      (let ((var (aref var-vec i)))
        (let ((raw-weight (/ 1.0 (+ var *base-variance*))))
          (setf (aref weight-vec i)
                (min raw-weight *max-weight-limit*)))))

    (list mean-vec weight-vec)))

(defun calc-weighted-cosine-similarity (input-vec mean-vec weight-vec)
  (let* ((w-x (v-mul weight-vec input-vec))
         (w-mu (v-mul weight-vec mean-vec))
         (numerator (v. w-x mean-vec))
         (denom-input (sqrt (abs (v. w-x input-vec))))
         (denom-mean (sqrt (abs (v. w-mu mean-vec)))))

    (if (or (< denom-input *epsilon*) (< denom-mean *epsilon*))
        0.0
      (/ numerator (* denom-input denom-mean)))))

(defun calc-cosine-similarity (v1 v2)
  (let ((val (v. v1 v2))
        (norm1 (norm v1))
        (norm2 (norm v2)))
    (if (or (< norm1 *epsilon*) (< norm2 *epsilon*))
        0.0
        (/ val (* norm1 norm2)))))

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

;; --- 4. ファイル入出力 ---
(defun save-raw-data (filename)
  (if *raw-training-data*
      (progn
        (dump-loadable-structure filename *raw-training-data*)
        (format t "[System] Saved RAW DATA to ~A~%" filename))
      (warn "[System] No raw data to save!~%")))

(defun load-raw-data (filename)
  (if (probe-file filename)
      (progn
        (load filename)
        (format t "[System] Loaded RAW DATA from ~A.~%" filename)
        (format t "[System] Please press 't' to re-train with current parameters.~%"))
      (warn "[System] File ~A not found!~%" filename)))

;; --- 5. ROS ---
(defun get-class-id (class-sym)
  (case class-sym
    (:class-1 1)
    (:class-2 2)
    (:class-3 3)
    (:class-4 4)
    (:class-5 5)
    (:class-6 6)
    (t -1)))

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
 [s]  : Start Inference Mode
 [m]  : Start Monitoring Mode
 [w/l/r]: Save/Load/Reset data
 [q]  : Quit
----------------------------------------------
")
  (let ((running t)
        (inference-mode nil)
        (monitoring-mode nil)
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
           (train-models)
           (format t "--- Training Done ---~%"))
          (#\s
           (setq inference-mode (not inference-mode))
           (setq monitoring-mode nil)
           (format t "~%Inference Mode: ~A~%" (if inference-mode "ON" "OFF")))
          (#\m
           (setq *monitoring-target* (copy-object *current-sensor-vector*))
           (setq monitoring-mode (not monitoring-mode))
           (setq inference-mode nil)
           (format t "~%Monitoring Mode: ~A~%" (if monitoring-mode "ON" "OFF")))
          (#\r
           (setq *raw-training-data* nil)
           (setq *trained-models* nil)
           (format t "~%All data cleared.~%"))
          (#\w (save-raw-data "training_data.l"))
          (#\l (load-raw-data "training_data.l"))
          (#\q (setq running nil))
          ))
      ;; --- B. 推論 ---
      (let ((publish-val 0)
            (publish-score 0.0))
        (cond
         ;; Monitoring Mode
         (monitoring-mode
          (if *monitoring-target*
              (progn
                (setq publish-score (calc-cosine-similarity *current-sensor-vector* *monitoring-target*))
                (setq publish-val -1)
                (format t "Monitoring... Score: ~5,3F (Target Locked)~%" publish-score))
            (warn "No target locked yet!~%")))
         ;; Inference Mode
         ((and inference-mode *trained-models*)
          (let ((input-norm (norm *current-sensor-vector*)))
            (cond
             ;; 接触なし
             ((< input-norm *contact-threshold*)
              (setq publish-val 0)
              (setq publish-score 0.0)
              (format t "Status: No Contact~%"))
             ;; 接触あり
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
                      (format t "Classify: ~A (Score: ~5,3F)~%" best-class max-score))
                  (progn
                    (setq publish-val -1)
                    (format t "Classify: UNKNOWN (Score: ~5,3F)~%" max-score)))
                )))))
         (t
          )
         )
        ;; --- Publish処理 ---
        (when (or monitoring-mode inference-mode)
          (send *pub-msg* :data publish-val)
          (ros::publish "hand_class" *pub-msg*)
          (send *score-msg* :data publish-score)
          (ros::publish "classification_score" *score-msg*)
          (finish-output *standard-output*))
        )

      (ros::duration-sleep 0.1)))
  (format t "~%Exiting.~%"))

(main-loop)
