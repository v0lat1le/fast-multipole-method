struct ShaderData {
	float2* pos;
	float* mass;
	float3 color;
	int mode;
};

struct VertexOutput {
    float3 color;
    float4 sv_position : SV_Position;
	float pointSize : SV_PointSize;
};

[shader("vertex")]
VertexOutput vertMain(uint vertex_index : SV_VertexID, uniform ShaderData data) {
    VertexOutput output;
    output.sv_position = float4(data.pos[vertex_index]*2.0-1.0, 0.0, 1.0);
    output.color = data.color;
	output.pointSize = clamp(2.0*sqrt(data.mass[vertex_index]), 1.0, 10.0);
    return output;
}

struct FragmentInput {
    float3 color;
	float2 pointCoord : SV_PointCoord;
}

[shader("fragment")]
float4 fragMain(FragmentInput frag, uniform ShaderData data) : SV_Target {
	if (data.mode == 0) {
		float2 uv = frag.pointCoord - float2(0.5, 0.5);
		if (length(uv) > 0.5) {
			discard;
		}
	}
    float3 color = frag.color;
    return float4(color, 1.0);
}
