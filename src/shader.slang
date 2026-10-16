struct ShaderData {
	float2* pos;
	float* mass;
	float3 color;
	float zoom;
	float2 center;
	int mode;
};

[[vk::binding(0, 0)]] ConstantBuffer<ShaderData> data;

struct VertexOutput {
    float3 color;
    float4 position : SV_Position;
    float2 uv : TEXCOORD0;
	float point_size : SV_PointSize;
};

[shader("vertex")]
VertexOutput vertMain(uint vertex_index : SV_VertexID) {
    VertexOutput output;
	if (data.mode == 2) {
		output.position = float4(data.pos[2*vertex_index], 0.0, 1.0);
		output.uv = data.pos[2*vertex_index+1];
		output.color = data.color;
		return output;
	}
    output.position = float4((data.pos[vertex_index]-data.center)*data.zoom, 0.0, 1.0);
    output.color = data.color;
	if (data.mode == 0) {
	    output.point_size = clamp(2.0*sqrt(data.mass[vertex_index]), 1.0, 10.0);
	}
    return output;
}

struct FragmentInput {
    float3 color;
	float2 point_coord : SV_PointCoord;
    float2 uv : TEXCOORD0;
}

[ForceInline]
T gpu_texture<T : IOpaqueDescriptor>(uint index)
{
    return getDescriptorFromHandle(DescriptorHandle<T>(uint2(index, 0)));
}

[ForceInline]
T gpu_sampler<T : IOpaqueDescriptor = SamplerState>(uint index)
{
    return getDescriptorFromHandle(DescriptorHandle<T>(uint2(index, 0)));
}

[shader("fragment")]
float4 fragMain(FragmentInput frag) : SV_Target {
	if (data.mode == 0) {
		float2 uv = frag.point_coord - float2(0.5, 0.5);
		if (length(uv) > 0.5) {
			discard;
		}
	}
	if (data.mode == 2) {
		Texture2D<float> texture = gpu_texture<Texture2D<float>>(0);
		SamplerState sampler = gpu_sampler(0);
		float texel = texture.Sample(sampler, frag.uv);
		return float4(frag.color, texel);
	}
	return float4(frag.color, 1.0);
}
