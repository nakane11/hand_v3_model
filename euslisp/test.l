#!/usr/bin/env roseus

;; --- 1. クラス定義 ---

(defclass hand-holding
  :slots ())

(defmethod hand-holding
  (:init ()
   (unless (ros::ok) (ros::roseus "hand_holding"))
   (ros::advertise "hand_angle" std_msgs::Float32MultiArray 1)
   (format t "[System] Hand Interface Initialized.~%")
  )

  (:publish-angle (angle-vec)
   (let ((pub-msg (instance std_msgs::Float32MultiArray :init)))
     (send pub-msg :data angle-vec)
     (ros::publish "hand_angle" pub-msg)))

  ;; --- 認識と実行メソッド (新規追加) ---
  (:recognize-and-act ()
   (format t ">> [Recognition Mode] Collecting data... (Press 'c' to Cancel)~%")
   (let ((history nil)       ;; 履歴バッファ
         (looping t)         ;; ループフラグ
         (recognized-id nil) ;; 判定されたID
         (counts (instantiate integer-vector 7))) ;; カウント用 (インデックス1-6を使用)

     (while (and (ros::ok) looping)
       ;; 1. 中断チェック (非ブロッキング)
       (when (select-stream (list *standard-input*) 0.001)
         (let ((char (read-char *standard-input*)))
           (when (or (eq char #\c) (eq char #\q))
             (format t ">> Recognition CANCELLED by user.~%")
             (return-from :recognize-and-act nil))))

       ;; 2. データ取得 (one-shot-subscribe)
       ;; timeoutを100msに設定し、データが来なくてもループを回してキー入力を確認できるようにする
       (let ((msg (one-shot-subscribe "hand_class" std_msgs::Int8)))
         (when msg
           (let ((val (send msg :data)))
               ;; 履歴に追加
               (push val history)
               ;; 長さが15を超えたら古いものを捨てる
               (if (> (length history) 15)
                   (setq history (subseq history 0 15)))

               ;; 履歴が15個溜まったら判定を行う
               (when (= (length history) 15)
                 ;; カウントリセット
                 (fill counts 0)
                 ;; 集計
                 (dolist (h history)
                   (when (and (>= val 1) (<= val 6))
                     (incf (aref counts h))))
                 ;; (format t "Current buffer: ~A~%" history)

                 ;; 閾値判定 (10個以上)
                 (dotimes (i 6)
                   (let ((target-id (+ i 1)))
                     (when (>= (aref counts target-id) 10)
                       (setq recognized-id target-id)
                       (setq looping nil) ;; ループ終了
                       (return)))) ;; dotimes脱出
                 )
               ))
         ))
        ;; End while

     ;; 3. 動作実行
     (when recognized-id
       (format t ">> DETECTED Class [~A] (>=10/15 matches). Executing motion...~%" recognized-id)
       (case recognized-id
         (1 (send self :five-finger-hook))
         (2 (send self :interlocked-fingers))
         (3 (send self :finger-hold))
         (4 (send self :mutual-palm-grasp))
         (5 (send self :finger-cradle-grasp))
         (6 (send self :four-finger-hook))
         (t (warn "Unknown ID detected.~%")))
       (format t ">> Motion Complete.~%")
       )
     ))

  ;; --- 各モーション定義 (変更なし) ---
  (:five-finger-hook ()
   (format t ">> Executing: Five Finger Hook~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.1)
   (send self :publish-angle (float-vector 10.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector 10.0 -40.0 40.0 40.0 -40.0 35.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector 10.0 -35.0 35.0 35.0 -35.0 35.0 1.5))
   (ros::duration-sleep 2.0)
   (send self :publish-angle (float-vector 10.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   )

  (:interlocked-fingers ()
   (format t ">> Executing: Interlocked Fingers~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.1)
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 -0.2))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector -20.0 -40.0 40.0 40.0 -40.0 35.0 -0.2))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector -20.0 -35.0 35.0 35.0 -35.0 35.0 -0.2))
   (ros::duration-sleep 2.0)
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 -0.2))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.5)
   )

  (:finger-hold ()
   (format t ">> Executing: Finger Hold~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.1)
   (send self :publish-angle (float-vector 10.0 60.0 -60.0 10.0 60.0 -60.0 0.2))
   (ros::duration-sleep 3.5)
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   )

  (:mutual-palm-grasp ()
   (format t ">> Executing: Mutual Palm Grasp~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.1)
   (send self :publish-angle (float-vector 50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector 50.0 -40.0 40.0 40.0 -40.0 35.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector 50.0 -35.0 35.0 35.0 -35.0 35.0 1.5))
   (ros::duration-sleep 2.0)
   (send self :publish-angle (float-vector 50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   )

  (:finger-cradle-grasp ()
   (format t ">> Executing: Finger Cradle Grasp~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.1)
   (send self :publish-angle (float-vector -50.0 -10.0 10.0 10.0 -10.0 -60.0 1.5))
   (ros::duration-sleep 4.0)
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   )

  (:four-finger-hook ()
   (format t ">> Executing: Four Finger Hook~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.1)
   (send self :publish-angle (float-vector 10.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector 10.0 -25.0 25.0 25.0 -25.0 25.0 1.5))
   (ros::duration-sleep 4.0)
   (send self :publish-angle (float-vector 10.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   )
  )


;; --- 2. メインループ ---

(defun main-loop ()
  (format t "
=== Hand Motion Controller ===
 [1] Five Finger Hook
 [2] Interlocked Fingers
 [3] Finger Hold
 [4] Mutual Palm Grasp
 [5] Finger Cradle Grasp
 [6] Four Finger Hook
 [r] Start Recognition Mode (Wait for result)
 [q] Quit
------------------------------
")

  (let ((hand (instance hand-holding :init))
        (running t)
        (key nil))

    (while (and (ros::ok) running)
      (ros::spin-once)

      ;; キー入力チェック
      (when (select-stream (list *standard-input*) 0.001)
        (setq key (read-char *standard-input*))

        (case key
          (#\1 (send hand :five-finger-hook))
          (#\2 (send hand :interlocked-fingers))
          (#\3 (send hand :finger-hold))
          (#\4 (send hand :mutual-palm-grasp))
          (#\5 (send hand :finger-cradle-grasp))
          (#\6 (send hand :four-finger-hook))
          (#\r (send hand :recognize-and-act)) ;; 認識モード呼び出し
          (#\q
           (format t "Exiting...~%")
           (setq running nil))
          (#\newline )
          (t (format t "Unknown key: ~C~%" key))
          )

        (unless (member key '(#\newline))
           (format t "Ready. [1-6/r/q] > "))
        )

      (unix:usleep 10000)
      )
    ))

(main-loop)
