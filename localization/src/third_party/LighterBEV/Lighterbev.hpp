#pragma once
#include <torch/torch.h>
#include <torch/script.h>
#include <iostream>

// -------------------- BasicBlock --------------------
struct BasicBlockImpl : public torch::nn::Module {
    torch::nn::Conv2d      conv1{nullptr}, conv2{nullptr};
    torch::nn::BatchNorm2d bn1{nullptr},  bn2{nullptr};
    torch::nn::Sequential  downsample{nullptr};
    torch::nn::ReLU        relu{nullptr};
    int stride_;

    BasicBlockImpl(int inplanes, int planes, int stride = 1, bool use_downsample = false)
        : stride_(stride)
    {
        using namespace torch::nn;

        conv1 = register_module(
            "conv1",
            Conv2d(Conv2dOptions(inplanes, planes, 3)
                       .stride(stride)
                       .padding(1)
                       .bias(false))
        );
        bn1 = register_module("bn1", BatchNorm2d(planes));
        relu = register_module("relu", ReLU(ReLUOptions().inplace(true)));

        conv2 = register_module(
            "conv2",
            Conv2d(Conv2dOptions(planes, planes, 3)
                       .stride(1)
                       .padding(1)
                       .bias(false))
        );
        bn2 = register_module("bn2", BatchNorm2d(planes));

        if (use_downsample) {
            downsample = register_module(
                "downsample",
                torch::nn::Sequential(
                    Conv2d(Conv2dOptions(inplanes, planes, 1)
                               .stride(stride)
                               .bias(false)),
                    BatchNorm2d(planes)
                )
            );
        }
    }

    torch::Tensor forward(torch::Tensor x) {
        auto identity = x;

        auto out = conv1->forward(x);
        out = bn1->forward(out);
        out = relu->forward(out);

        out = conv2->forward(out);
        out = bn2->forward(out);

        if (downsample) {
            identity = downsample->forward(x);
        }

        out += identity;
        out = relu->forward(out);
        return out;
    }
};
TORCH_MODULE(BasicBlock);


// -------------------- Layer1: 对应 Python encoder[4] --------------------
struct Layer1Impl : public torch::nn::Module {
    BasicBlock b0{nullptr}, b1{nullptr}, b2{nullptr};

    Layer1Impl() {
        // 这里名字用 "0" "1" "2"，保证 key 是 rem.encoder.4.0.xxx
        b0 = register_module("0", BasicBlock(64, 64, 1, false));
        b1 = register_module("1", BasicBlock(64, 64, 1, false));
        b2 = register_module("2", BasicBlock(64, 64, 1, false));
    }

    torch::Tensor forward(torch::Tensor x) {
        x = b0->forward(x);
        x = b1->forward(x);
        x = b2->forward(x);
        return x;
    }
};
TORCH_MODULE(Layer1);


// -------------------- Layer2: 对应 Python encoder[5] --------------------
struct Layer2Impl : public torch::nn::Module {
    BasicBlock b0{nullptr}, b1{nullptr}, b2{nullptr}, b3{nullptr};

    Layer2Impl() {
        // 5.0: 64->128, stride=2, 带 downsample
        b0 = register_module("0", BasicBlock(64, 128, 2, true));
        // 5.1~5.3: 128->128, stride=1
        b1 = register_module("1", BasicBlock(128, 128, 1, false));
        b2 = register_module("2", BasicBlock(128, 128, 1, false));
        b3 = register_module("3", BasicBlock(128, 128, 1, false));
    }

    torch::Tensor forward(torch::Tensor x) {
        x = b0->forward(x);
        x = b1->forward(x);
        x = b2->forward(x);
        x = b3->forward(x);
        return x;
    }
};
TORCH_MODULE(Layer2);


// -------------------- 构造 encoder：结构 & key 和 Python 一致 --------------------
inline torch::nn::Sequential make_encoder_resnet34_front() {
    using namespace torch::nn;

    torch::nn::Sequential encoder;

    // 0: Conv2d(3,64,7x7,stride=2,pad=3)
    encoder->push_back(
        Conv2d(Conv2dOptions(3, 64, 7)
                   .stride(2)
                   .padding(3)
                   .bias(false))
    );
    // 1: BN(64)
    encoder->push_back(BatchNorm2d(64));
    // 2: ReLU
    encoder->push_back(ReLU(ReLUOptions().inplace(true)));
    // 3: MaxPool(3, stride=2, padding=1)
    encoder->push_back(MaxPool2d(MaxPool2dOptions(3).stride(2).padding(1)));

    // 4: layer1
    encoder->push_back(Layer1());
    // 5: layer2
    encoder->push_back(Layer2());

    return encoder;
}

/*********************************
 * NetVLADImpl
 *********************************/
class NetVLADImpl : public torch::nn::Module {
public:
    NetVLADImpl(int pca_dim = 32,
                bool use_pca = true,
                int num_clu = 64,
                int dim = 128)
        : num_clusters_(num_clu),
          dim_(dim),
          pca_dim_(pca_dim),
          use_pca_(use_pca)
    {
        int in_c = use_pca_ ? pca_dim_ : dim_;

        conv_ = register_module(
            "conv",
            torch::nn::Conv2d(torch::nn::Conv2dOptions(in_c, num_clusters_, 1).bias(false)));

        centroids_ = register_parameter("centroids", torch::rand({num_clusters_, in_c}));
    }

    torch::Tensor forward(const torch::Tensor& x)
    {
        const auto N = x.size(0), C = x.size(1);
        auto xf = x.view({N, C, -1});  // N,C,HW

        auto soft = conv_->forward(x).view({N, num_clusters_, -1});
        soft = torch::softmax(soft, 1);

        auto vlad = torch::zeros({N, num_clusters_, C}, x.options());
        for (int k = 0; k < num_clusters_; ++k) {
            auto residual = xf - centroids_[k].view({1, -1, 1});
            residual *= soft.select(1, k).unsqueeze(1);
            vlad.select(1, k) = residual.sum(-1);
        }

        vlad = torch::nn::functional::normalize(
            vlad, torch::nn::functional::NormalizeFuncOptions().p(2).dim(2));

        vlad = vlad.view({N, -1});
        vlad = torch::nn::functional::normalize(
            vlad, torch::nn::functional::NormalizeFuncOptions().p(2).dim(1));
        return vlad;
    }

private:
    int num_clusters_, dim_, pca_dim_;
    bool use_pca_;
    float alpha{1.f};

    torch::nn::Conv2d conv_{nullptr};
    torch::Tensor centroids_;
};


TORCH_MODULE(NetVLAD);

/*********************************
 * REMImpl
 *********************************/
class REMImpl : public torch::nn::Module {
public:
    REMImpl(bool from_scratch, int rotations,
            int pca_dim = 32, bool use_pca = true,
            int dim = 128, bool init_pca = false)
        : rotations(rotations), use_pca_(use_pca), init_pca_(init_pca)
    {
        (void)from_scratch;
        encoder = register_module("encoder", make_encoder_resnet34_front());

        angles = -torch::arange(0, 359.00001, 360.0 / rotations) / 180.0 * M_PI;

        pca_mean = register_parameter("pca_mean", torch::rand({dim}));
        pca_rot  = register_parameter("pca_rot",  torch::rand({pca_dim, dim}));
    }

    std::tuple<torch::Tensor, torch::Tensor> forward(torch::Tensor x) {
        std::vector<torch::Tensor> equ_features;  // 用于存储每个旋转的特征
        auto batch_size = x.size(0);  // 获取批量大小
        std::vector<int64_t> im1_init_size;


        
        for (int i = 0; i < angles.size(0); ++i) {
            // 创建旋转矩阵（使用负角度进行输入图像的旋转）
            auto aff = torch::zeros({batch_size, 2, 3}).to(x.device());
            aff.select(2, 0).select(1, 0) = torch::cos(-angles[i]);
            aff.select(2, 0).select(1, 1) = torch::sin(-angles[i]);
            aff.select(2, 1).select(1, 0) = -torch::sin(-angles[i]);
            aff.select(2, 1).select(1, 1) = torch::cos(-angles[i]);

            // 根据旋转矩阵生成新的采样网格
            auto grid = torch::nn::functional::affine_grid(aff, x.sizes(), /*align_corners=*/true);
            // Equivalent to Python F.grid_sample(mode='bicubic',
            // padding_mode='zeros', align_corners=True). The C++ options
            // struct has no kBicubic enum in any stock libtorch release, so
            // call the underlying dispatcher directly (mode=2 is bicubic).
            auto warped_im = torch::grid_sampler(x, grid, 2, 0, true);

            // 提取特征
            auto out = encoder->forward(warped_im);
            // 第一次计算时，记录输出的初始尺寸
            if (i == 0) {
                auto sz = out.sizes();            
                im1_init_size.assign(sz.begin(), sz.end());
            }
            
            // 创建逆旋转矩阵（使用正角度）
            auto aff_inv = torch::zeros({batch_size, 2, 3}).to(x.device());
            aff_inv.select(2, 0).select(1, 0) = torch::cos(angles[i]);
            aff_inv.select(2, 0).select(1, 1) = torch::sin(angles[i]);
            aff_inv.select(2, 1).select(1, 0) = -torch::sin(angles[i]);
            aff_inv.select(2, 1).select(1, 1) = torch::cos(angles[i]);
            
            // 根据逆旋转矩阵生成新的采样网格
            auto grid_inv = torch::nn::functional::affine_grid(aff_inv, im1_init_size, /*align_corners=*/true);
            
            auto warped_out = torch::grid_sampler(out, grid_inv, 2, 0, true);
            // 将当前旋转后的特征添加到列表中
            equ_features.push_back(warped_out.unsqueeze(-1));
        }
        
        // 将所有旋转后的特征拼接起来
        auto equ_features_tensor = torch::cat(equ_features, -1);  // B C H W R
        auto equ_max = std::get<0>(torch::max(equ_features_tensor, -1, /*keepdim=*/false));

        // 创建单位矩阵以用于上采样
        auto aff_identity = torch::zeros({batch_size, 2, 3}).to(x.device());
        aff_identity.select(2, 0).select(1, 0) = 1;
        aff_identity.select(2, 0).select(1, 1) = 0;
        aff_identity.select(2, 1).select(1, 0) = 0;
        aff_identity.select(2, 1).select(1, 1) = 1;

        auto grid1 = torch::nn::functional::affine_grid(aff_identity, torch::IntArrayRef({batch_size, equ_max.size(1), x.size(2) / 4, x.size(3) / 4}), /*align_corners=*/true);
        auto out1 = torch::grid_sampler(equ_max, grid1, 2, 0, true);
        out1 = torch::nn::functional::normalize(out1, /*dim=*/torch::nn::functional::NormalizeFuncOptions().dim(1));  // 归一化 out1
        
        if (use_pca_ && init_pca_ )
        {
            // 1. 获取形状
            auto sizes = out1.sizes();                // {N, C, H, W}
            int64_t N = sizes[0],
                    C = sizes[1],
                    H = sizes[2],
                    W = sizes[3];

            // 2. 展平到 N × C × (H*W)
            out1 = out1.view({N, C, -1});             // (N, C, HW)

            // 3. 做均值中心化：broadcast (1, C, 1)
            out1 = out1 - pca_mean.view({1, -1, 1});

            auto rot_T = pca_rot.transpose(0, 1);    // (C, pca_dim_)
            out1 = torch::matmul(out1.permute({0, 2, 1}), rot_T)  // (N, HW, pca_dim_)
                    .permute({0, 2, 1});                       // (N, pca_dim_, HW)

            // 5. 还原成 N × pca_dim_ × H × W
            C = out1.size(1);                         // pca_dim_
            out1 = out1.view({N, C, H, W});
        }

        auto grid2 = torch::nn::functional::affine_grid(aff_identity, {batch_size, equ_max.size(1), x.size(2), x.size(3)}, /*align_corners=*/true);
        auto out2 = torch::grid_sampler(equ_max, grid2, 2, 0, true);
        out2 = torch::nn::functional::normalize(out2, /*dim=*/torch::nn::functional::NormalizeFuncOptions().dim(1));  // 归一化 out2

        return std::make_tuple(out1, out2);
    }

private:
    int rotations;
    bool use_pca_;
    bool init_pca_;

    torch::Tensor angles;
    torch::Tensor pca_mean, pca_rot;

    torch::nn::Sequential encoder{nullptr};
};

TORCH_MODULE(REM);

/*********************************
 * REINImpl
 *********************************/
class REINImpl : public torch::nn::Module {
public:
    std::string model_path;

    REINImpl()
    {
        rem = register_module("rem", REM(false, 8, 32, true, 128, true));
        pooling = register_module("pooling", NetVLAD(32, true, 64, 128));
    }

    std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> forward(torch::Tensor x)
    {
        auto rem_out = rem->forward(x);
        auto out1 = std::get<0>(rem_out);
        auto local_feats = std::get<1>(rem_out);
        auto global_desc = pooling->forward(out1);
        return {out1, local_feats, global_desc};
    }


    void load_state_dict(const std::string &path) {
        torch::NoGradGuard no_grad;

        // 1) 读整个文件到内存
        std::ifstream file(path, std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "[REINImpl::load_state_dict] cannot open file: " << path << std::endl;
            return;
        }
        std::vector<char> data((std::istreambuf_iterator<char>(file)),
                            std::istreambuf_iterator<char>());
        file.close();

        if (data.empty()) {
            std::cerr << "[REINImpl::load_state_dict] file is empty: " << path << std::endl;
            return;
        }

        // 2) 用 libtorch 自带 unpickler 反序列化
        torch::IValue iv = torch::pickle_load(data);

        if (!iv.isGenericDict()) {
            std::cerr << "[REINImpl::load_state_dict] ivalue is not GenericDict "
                    << "(did you save with torch.save(dict(model.state_dict()), ...)?).\n";
            std::cerr << "  iv.isNone() = " << iv.isNone() << std::endl;
            return;
        }
        
        auto generic_dict = iv.toGenericDict();  // c10::Dict<IValue, IValue>

        // 3) 把 GenericDict 转成 std::unordered_map<std::string, Tensor>
        std::unordered_map<std::string, at::Tensor> ckpt;
        ckpt.reserve(generic_dict.size());
        for (auto &kv : generic_dict) {
            std::string key = kv.key().toStringRef();
            ckpt.emplace(key, kv.value().toTensor());
        }

        // 4) 取出当前模块的参数和 buffer
        auto params  = this->named_parameters(/*recurse=*/true);
        auto buffers = this->named_buffers(/*recurse=*/true);

        // 覆盖参数
        for (auto &p : params) {
            const auto &k = p.key();
            auto it = ckpt.find(k);
            if (it != ckpt.end()) {
                p.value().copy_(it->second);
            } else {
                std::cerr << "[REINImpl::load_state_dict] missing parameter in checkpoint: "
                        << k << std::endl;
            }
        }

        // 覆盖 buffers（BN running_mean / running_var 等）
        for (auto &b : buffers) {
            const auto &k = b.key();
            auto it = ckpt.find(k);
            if (it != ckpt.end()) {
                b.value().copy_(it->second);
            } else {
                std::cerr << "[REINImpl::load_state_dict] missing buffer in checkpoint: "
                        << k << std::endl;
            }
        }

        // 5) 可选：打印 checkpoint 里多余的 key（方便对齐 Python / C++ 命名）
        for (const auto &kv : ckpt) {
            const auto &k = kv.first;
            if (!params.contains(k) && !buffers.contains(k)) {
                std::cerr << "[REINImpl::load_state_dict] unexpected key in checkpoint: "
                        << k << std::endl;
            }
        }
    }
    


    void save_state_dict(const std::string &model_path)
    {
        try
        {
            torch::Dict<std::string, torch::Tensor> dict;

            for (const auto &p : this->named_buffers())
            {
                dict.insert(p.key(), p.value());
            }

            for (const auto &p : this->named_parameters())
            {
                dict.insert(p.key(), p.value());
            }
            
            std::vector<char> state_dict = torch::pickle_save(dict);
            std::ofstream file(model_path, std::fstream::out | std::ios::binary);
            file.write(state_dict.data(), state_dict.size());
            file.close();
        }
        catch (const c10::Error &e)
        {
            std::cerr << e.what() << "\n";
            std::cerr << "Error saving the state_dict\n";
        }
    }


private:
    REM rem{nullptr};
    NetVLAD pooling{nullptr};
};


TORCH_MODULE(REIN);
