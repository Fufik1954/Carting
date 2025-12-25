using UnityEngine;

public class CarSuspension : MonoBehaviour
{
    [Header("Suspension Points")]
    [SerializeField] private Transform fl;
    [SerializeField] private Transform fr;
    [SerializeField] private Transform rl;
    [SerializeField] private Transform rr;

    [Header("Suspension Settings")]
    [SerializeField] private float restLength = 0.4f;
    [SerializeField] private float springTravel = 0.2f;
    [SerializeField] private float springStiffness = 20000f;
    [SerializeField] private float damperStiffness = 3500f;
    [SerializeField] private float wheelRadius = 0.35f;

    [Header("Anti-Roll Bar")]
    [SerializeField] private float frontAntiRollStiffness = 8000f;
    [SerializeField] private float rearAntiRollStiffness = 6000f;

    // Данные для телеметрии
    private float flDistance, frDistance, rlDistance, rrDistance;
    private float flCompression, frCompression, rlCompression, rrCompression;
    private float flSpringForce, frSpringForce, rlSpringForce, rrSpringForce;
    private float flDamperForce, frDamperForce, rlDamperForce, rrDamperForce;

    private Rigidbody rb;
    private LayerMask groundLayer;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        groundLayer = LayerMask.GetMask("Default");
    }

    private void FixedUpdate()
    {
        SimulateWheel(fl, ref flCompression, out flDistance, out flSpringForce, out flDamperForce);
        SimulateWheel(fr, ref frCompression, out frDistance, out frSpringForce, out frDamperForce);
        SimulateWheel(rl, ref rlCompression, out rlDistance, out rlSpringForce, out rlDamperForce);
        SimulateWheel(rr, ref rrCompression, out rrDistance, out rrSpringForce, out rrDamperForce);

        ApplyAntiRollBars();
    }

    private void SimulateWheel(Transform pivot, ref float compression,
        out float distance, out float springForce, out float damperForce)
    {
        distance = 0f;
        springForce = 0f;
        damperForce = 0f;

        if (pivot == null) return;

        Vector3 origin = pivot.position;
        Vector3 direction = -pivot.up;
        float maxDist = restLength + springTravel + wheelRadius;

        if (Physics.Raycast(origin, direction, out RaycastHit hit, maxDist, groundLayer))
        {
            distance = hit.distance;

            float currentLength = hit.distance - wheelRadius;
            currentLength = Mathf.Clamp(currentLength,
                restLength - springTravel,
                restLength + springTravel);

            float newCompression = restLength - currentLength;

            springForce = newCompression * springStiffness;

            float compressionVelocity = (newCompression - compression) / Time.fixedDeltaTime;
            damperForce = compressionVelocity * damperStiffness;

            compression = newCompression;

            float totalForce = springForce + damperForce;
            rb.AddForceAtPosition(pivot.up * totalForce, pivot.position, ForceMode.Force);
        }
        else
        {
            compression = 0f;
            distance = maxDist;
        }
    }

    private void ApplyAntiRollBars()
    {
        float frontDiff = flCompression - frCompression;
        float frontForce = frontDiff * frontAntiRollStiffness;

        if (flCompression > -0.0001f)
            rb.AddForceAtPosition(-transform.up * frontForce, fl.position, ForceMode.Force);
        if (frCompression > -0.0001f)
            rb.AddForceAtPosition(transform.up * frontForce, fr.position, ForceMode.Force);

        float rearDiff = rlCompression - rrCompression;
        float rearForce = rearDiff * rearAntiRollStiffness;

        if (rlCompression > -0.0001f)
            rb.AddForceAtPosition(-transform.up * rearForce, rl.position, ForceMode.Force);
        if (rrCompression > -0.0001f)
            rb.AddForceAtPosition(transform.up * rearForce, rr.position, ForceMode.Force);
    }

    private void OnGUI()
    {
        if (!Application.isPlaying) return;

        float screenWidth = Screen.width;
        float x = screenWidth - 320; 
        float y = 10;
        float lineHeight = 20;

        float speed = rb.linearVelocity.magnitude;
        float dragForce = 0.5f * 1.225f * 0.9f * 0.6f * speed * speed;
        GUI.Label(new Rect(x, y, 300, lineHeight), $"Сопротивление: {dragForce:F0} Н");
        y += lineHeight;

        float downforce = speed * 70f;
        GUI.Label(new Rect(x, y, 300, lineHeight), $"Прижим крыла: {downforce:F0} Н");
        y += lineHeight;

        GUI.Label(new Rect(x, y, 300, lineHeight), "Силы подвески (Н):");
        y += lineHeight;

        float flTotal = flSpringForce + flDamperForce;
        float frTotal = frSpringForce + frDamperForce;
        float rlTotal = rlSpringForce + rlDamperForce;
        float rrTotal = rrSpringForce + rrDamperForce;

        GUI.Label(new Rect(x, y, 300, lineHeight), $"   FL: {flTotal:F0}  FR: {frTotal:F0}");
        y += lineHeight;
        GUI.Label(new Rect(x, y, 300, lineHeight), $"   RL: {rlTotal:F0}  RR: {rrTotal:F0}");
        y += lineHeight;

        GUI.Label(new Rect(x, y, 300, lineHeight), "Высота колес (м):");
        y += lineHeight;

        GUI.Label(new Rect(x, y, 300, lineHeight), $"   FL: {flDistance:F3}  FR: {frDistance:F3}");
        y += lineHeight;
        GUI.Label(new Rect(x, y, 300, lineHeight), $"   RL: {rlDistance:F3}  RR: {rrDistance:F3}");
        y += lineHeight;

        GUI.Label(new Rect(x, y, 300, lineHeight), "Сжатие подвески:");
        y += lineHeight;

        GUI.Label(new Rect(x, y, 300, lineHeight), $"   FL: {flCompression:F3}  FR: {frCompression:F3}");
        y += lineHeight;
        GUI.Label(new Rect(x, y, 300, lineHeight), $"   RL: {rlCompression:F3}  RR: {rrCompression:F3}");
        y += lineHeight;

        float comHeight = rb.worldCenterOfMass.y;
        GUI.Label(new Rect(x, y, 300, lineHeight), $"Высота ЦМ: {comHeight:F3} м");
    }
}