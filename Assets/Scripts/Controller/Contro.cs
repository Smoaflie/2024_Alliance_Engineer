using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class Contro : MonoBehaviour
{
    public Transform Target;
    public Transform Pitch;
    public Transform YawAndRoll;
    public Transform BigYaw;

    public Text coordinate;
    public Text debugText;

    public enum ControKeyCode
    {
        None = 0,
        translationFront = 1 << 0, // 0001
        translationBack = 1 << 1, // 0010
        translationLeft = 1 << 2, // 0100
        translationRight = 1 << 3,  // 1000
        rotationUp = 1 << 4, // 0001 0000
        rotationDown = 1 << 5, // 0010 0000
        rotationLeft = 1 << 6, // 0100 0000
        rotationRight = 1 << 7,  // 1000 0000
        speed_state = 1 << 8, // 0001 0000 0000
        heightUp = 1 << 9, // 0010 0000 0000
        heightDown = 1 << 10, // 0100 0000 0000
        YawRotateLeft = 1 << 11,  // 1000 0000 0000
        YawRotateRight = 1 << 12, // 0001 0000 0000 0000
        translateMode = 1 << 13, // 0010 0000 0000 0000
        rotateMode = 1 << 14, // 0100 0000 0000 0000
        ResetFlag = 1 << 15,  // 1000 0000 0000 0000
        EnableDecodeMode = 1 << 16,  // 0001 0000 0000 0000 0000
        CustomTargetContro = 1 << 17,  // 0010 0000 0000 0000 0000
        CustomPositionContro = 1 << 18,  // 0101 0000 0000 0000 0000
        EnableGenerateTarget = 1 << 19,  // 1000 0000 0000 0000 0000
        EnableChangeDecodeFunc = 1 << 20, // 0001 0000 0000 0000 0000 0000
        EnableCustomController = 1 << 21, // 0010 0000 0000 0000 0000 0000
    }

    public enum ArmControMode_
    {
        None = 0,
        TargetDecode = 1 << 0, // 0001
        ArmGenerate = 1 << 1, // 0010
        ArmCustomContro = 1 << 2, // 0010
        ArmReset = 1 << 3, // 0100
        MCU = 1 << 3, // 0100
    }

    public static ArmControMode_ ArmControMode;
    public static ControKeyCode _KeyCode;
    public static float[] encoder_Data = new float[3];
    public static Quaternion qua_rec = Quaternion.identity;
    
    private float Speed = 0.1f;
    private float rotationSpeed = 0.05f;
    void Start()
    {
        Cursor.visible = false;
        Cursor.lockState = CursorLockMode.Locked;
    }

    bool change_flag = false;
    // Update is called once per frame
    void Update()
    {   
        if(GetControKeyCode(ControKeyCode.EnableChangeDecodeFunc) && !change_flag)
        {
            change_flag = true;            
            Arm.ChangeDecodeFuncBool = !Arm.ChangeDecodeFuncBool;
        }else if(!GetControKeyCode(ControKeyCode.EnableChangeDecodeFunc))
        {
            change_flag = false;
        }

        UpdateSpeedAndRotation();

        Simulator.SimulatorEnable = false;
        if(GetControKeyCode(ControKeyCode.EnableCustomController))
        {
                Simulator.SimulatorEnable = true;
                Simulator.SimulatorPositionEnable = GetControKeyCode(ControKeyCode.CustomPositionContro);
                Simulator.SimulatorTargetEnable = GetControKeyCode(ControKeyCode.CustomTargetContro);

                ArmControMode = ArmControMode_.ArmCustomContro;

                HandleRotation(ControKeyCode.YawRotateLeft, Vector3.up, -1, true);
                HandleRotation(ControKeyCode.YawRotateRight, Vector3.up, 1, true);
                if(GetControKeyCode(ControKeyCode.YawRotateRight) || 
                        GetControKeyCode(ControKeyCode.YawRotateLeft))
                {
                    ArmControMode = ArmControMode_.ArmGenerate;    
                }

                if(GetControKeyCode(ControKeyCode.CustomPositionContro))
                {
                    HandleRotation(ControKeyCode.rotationRight, Vector3.up, 1);
                    HandleRotation(ControKeyCode.rotationLeft, Vector3.up, -1);
                    HandleRotation(ControKeyCode.rotationUp, Vector3.right, -1);
                    HandleRotation(ControKeyCode.rotationDown, Vector3.right, 1);
                }
        }
        else if(GetControKeyCode(ControKeyCode.EnableDecodeMode))
        {
            HandleTranslation(ControKeyCode.translationFront, Vector3.forward);
            HandleTranslation(ControKeyCode.translationBack, -Vector3.forward);
            HandleTranslation(ControKeyCode.translationLeft, -Vector3.right);
            HandleTranslation(ControKeyCode.translationRight, Vector3.right);

            HandleHeightChange(ControKeyCode.heightUp, Vector3.up);
            HandleHeightChange(ControKeyCode.heightDown, -Vector3.up);

            HandleRotation(ControKeyCode.rotationRight, Vector3.up, 1);
            HandleRotation(ControKeyCode.rotationLeft, Vector3.up, -1);
            HandleRotation(ControKeyCode.rotationUp, Vector3.right, -1);
            HandleRotation(ControKeyCode.rotationDown, Vector3.right, 1);
            HandleRotation(ControKeyCode.YawRotateLeft, Vector3.up, -1, true);
            HandleRotation(ControKeyCode.YawRotateRight, Vector3.up, 1, true);

            if(GetControKeyCode(ControKeyCode.YawRotateRight) || 
                GetControKeyCode(ControKeyCode.rotateMode) || 
                    GetControKeyCode(ControKeyCode.YawRotateLeft))
            {
                ArmControMode = ArmControMode_.ArmGenerate;    
            }
            else
            {
                ArmControMode = ArmControMode_.TargetDecode;
            }
        }
        else if(GetControKeyCode(ControKeyCode.EnableGenerateTarget))
        {
            ArmControMode = ArmControMode_.ArmGenerate;    
        }
        else
        {
            ArmControMode = ArmControMode_.None;
        }
        
        if(GetControKeyCode(ControKeyCode.ResetFlag))
        {
            ArmControMode = ArmControMode_.ArmReset;
        }
        
        
        // Target.position = Quaternion.AngleAxis(Input.GetAxis("Horizontal_L") * Time.deltaTime * Speed * 50, Vector3.up) * Target.position;
        // Target.position -= Vector3.forward * Input.GetAxis("Vertical_L") * Time.deltaTime * Speed * 0.5f;
        // Target.position += Vector3.up * Input.GetAxis("Triggers") * Speed * Time.deltaTime;

        // Target.Rotate(new Vector3(0, Input.GetAxis("Horizontal_R") * Speed * 10, 0), Space.World);
        // Target.Rotate(new Vector3(Input.GetAxis("Vertical_R") * Speed * 10, 0, 0), Space.Self);
    
        // Target.position += Vector3.up * Input.GetAxis("Mouse ScrollWheel") * 10 * Speed * Time.deltaTime;


        if(GetControKeyCode(ControKeyCode.translateMode))
            coordinate.text = "sucker";
        else
            coordinate.text = "world";
    }

    void UpdateSpeedAndRotation()
    {
        if (GetControKeyCode(ControKeyCode.speed_state))
        {
            Speed = 1;
            rotationSpeed = 0.5f;
        }
        else
        {
            Speed = 0.1f;
            rotationSpeed = 0.05f;
        }
    }
    void HandleTranslation(ControKeyCode direction, Vector3 vector)
    {
        if (GetControKeyCode(direction))
        {
            if (GetControKeyCode(ControKeyCode.translateMode))
            {
                Target.position += Target.TransformDirection(vector) * Time.deltaTime * Speed;
            }
            else
            {
                Target.position += vector * Time.deltaTime * Speed;
            }
        }
    }

    void HandleHeightChange(ControKeyCode direction, Vector3 vector)
    {
        if (GetControKeyCode(direction))
        {
            Target.position += vector * Time.deltaTime * Speed;
        }
    }

    void HandleRotation(ControKeyCode direction, Vector3 axis, float sign, bool isYaw = false)
    {
        if (GetControKeyCode(direction))
        {
            float rotationValue = sign * rotationSpeed * 10;
            Vector3 rotationVector = axis * rotationValue;

            if (isYaw)
            {
                HandleYawRotation(rotationVector);
            }
            else
            {
                if(GetControKeyCode(ControKeyCode.rotateMode))
                    HandleRegularRotation(rotationVector * 2.5f, direction);
                else
                    HandleTargetRotation(rotationVector);
            }
        }
    }

    void HandleYawRotation(Vector3 rotationVector)
    {   
        if (GetControKeyCode(ControKeyCode.rotateMode))
        {
            YawAndRoll.Rotate(rotationVector, Space.World);
        }
        else
        {
            BigYaw.Rotate(rotationVector, Space.World);
        }
    }

void HandleRegularRotation(Vector3 rotationVector, ControKeyCode direction)
    {
        if (GetControKeyCode(ControKeyCode.rotationLeft))
        {
            YawAndRoll.Rotate(new Vector3(0, 0, rotationSpeed * 10), Space.Self);
        }
        if (GetControKeyCode(ControKeyCode.rotationRight))
        {
            YawAndRoll.Rotate(new Vector3(0, 0, rotationSpeed * -10), Space.Self);
        }
        if (GetControKeyCode(ControKeyCode.rotationUp))
        {
            Pitch.Rotate(new Vector3(rotationSpeed * -30, 0, 0), Space.Self);
        }
        if (GetControKeyCode(ControKeyCode.rotationDown))
        {
            Pitch.Rotate(new Vector3(rotationSpeed * 30, 0, 0), Space.Self);
        }
    }
void HandleTargetRotation(Vector3 rotationVector)
{
    if(rotationVector.y != 0)
    {
        Target.Rotate(rotationVector, Space.World);
    }
    else if(rotationVector.x != 0)
    {
        Target.Rotate(rotationVector, Space.Self);
    }
}
    public static void SetControKeyCode(ControKeyCode target_keyCode)
    {
        _KeyCode |= target_keyCode;
    }
    public static void ResetControKeyCode(ControKeyCode target_keyCode)
    {
        _KeyCode &= ~target_keyCode;
    }
    public static void SwitchControKeyCode(ControKeyCode target_keyCode)
    {
        if (GetControKeyCode(target_keyCode))
        {
            _KeyCode &= ~target_keyCode;
        }
        else
        {
            _KeyCode |= target_keyCode;
        }
    }
    public static bool GetControKeyCode(ControKeyCode target_keyCode)
    {
        return (_KeyCode & target_keyCode) == target_keyCode;
    }
}
