% =========================================================================
% SCRIPT FOR 5-DOF LEG TRAJECTORIES (1 TIME COL + 5 JOINT COLS = 6 TOTAL)
% =========================================================================

% 1. تحميل البيانات الخام المولدة
load('jointAngs_generated.mat'); 

% 2. رسم مسارات المفاصل الخمسة للرجل اليسرى (الأعمدة من 2 إلى 6)
figure;
plot(jAngsL(:, 1), jAngsL(:, 2:7), 'LineWidth', 1.5);
title('Generated Left Leg Joint Trajectories');
xlabel('Time (seconds)');
ylabel('Joint Angle (radians)');
grid on;
legend('Joint 1 (Hip Yaw)', 'Joint 2 (Inter. Rev)', 'Joint 3 (Hip Pitch)', ...
       'Joint 4 (Knee Pitch)', 'Joint 5 (Ankle Pitch)', 'Location', 'best');

% 3. إنشاء مصفوفات فارغة بحجم المصفوفة الجديدة (6 أعمدة)
final_jAngsL = zeros(size(jAngsL));
final_jAngsR = zeros(size(jAngsR));

% 4. نسخ عمود الوقت (العمود الأول)
final_jAngsL(:, 1) = jAngsL(:, 1);
final_jAngsR(:, 1) = jAngsR(:, 1);

% 5. خريطة المفاصل الخمسة (من العمود 2 إلى 6):
% العمود 2: Joint 1 (Hip Yaw)
% العمود 3: Joint 2 (Intermediate)
% العمود 4: Joint 3 (Hip Pitch)
% العمود 5: Joint 4 (Knee Pitch)
% العمود 6: Joint 5 (Ankle Pitch)

final_jAngsL(:, 2:7) = jAngsL(:, 2:7);
final_jAngsR(:, 2:7) = jAngsR(:, 2:7);

% ملاحظة: إذا كنت تحتاج عكس إشارة مفصل معين لتوافق Simulink (مثل الركبة):
% final_jAngsL(:, 5) = -jAngsL(:, 5); 

% 6. حفظ الملف الجديد الجاهز لـ Simulink
jAngsL = final_jAngsL;
jAngsR = final_jAngsR;
save('jointAngs_simulink_ready.mat', 'jAngsL', 'jAngsR');
disp('Matrix successfully generated and ready for Simulink (5 Active Joints)!');