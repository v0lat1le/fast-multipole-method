struct VertexInput {
	float2* pos;
	float3 color;
};

struct VertexOutput {
    float3 color;
    float4 sv_position : SV_Position;
	float pointSize : SV_PointSize;
};

[shader("vertex")]
VertexOutput vertMain(uint vertex_index : SV_VertexID, uniform VertexInput data) {
    VertexOutput output;
    output.sv_position = float4(data.pos[vertex_index]*2.0-1.0, 0.0, 1.0);
    output.color = data.color;
	output.pointSize = 1.2;
    return output;
}

struct FragmentInput {
    float3 color;
    float4 sv_position : SV_Position;
}

[shader("fragment")]
float4 fragMain(FragmentInput inVert) : SV_Target {
    float3 color = inVert.color;
    return float4(color, 1.0);
}
