package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n100_u2_a8000m2_1
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=797.21,7765.80");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1396.12,7827.86");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2502.70,7766.90");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4498.73,7968.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4878.32,7866.97");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5647.23,7862.16");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=30.48,7526.51");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=381.38,7602.52");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2086.64,7737.91");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3037.41,7650.17");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3665.67,7636.56");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4335.78,7537.41");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5313.64,7724.78");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1003.72,7420.37");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1443.23,7418.17");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2526.68,7372.10");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4838.71,7433.86");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5376.79,7303.35");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=131.30,7181.95");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=543.52,7230.85");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1054.09,7012.79");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1712.88,7052.70");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2082.21,7044.58");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3250.63,7127.92");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3858.47,7179.42");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4475.45,7152.36");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5871.17,7218.02");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=226.41,6810.63");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3010.85,6813.74");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4129.44,6865.04");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4957.44,6850.16");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5309.91,6793.65");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=884.69,6613.34");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1515.95,6644.08");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2049.69,6691.75");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2666.96,6708.46");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3539.06,6658.19");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4508.24,6695.96");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5937.42,6722.68");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2378.98,6499.71");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3161.96,6432.56");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3819.20,6295.87");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4232.78,6446.25");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5049.83,6466.42");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=491.10,6206.61");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1129.77,6135.85");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1615.81,6258.34");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1954.17,6117.10");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2839.51,6020.29");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4462.17,6129.20");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5699.37,6184.49");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=257.18,5931.28");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=861.73,5813.72");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1520.61,5849.13");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3279.68,5887.90");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3647.63,5924.80");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4320.14,5778.32");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4921.64,5844.19");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2218.78,5674.44");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2737.86,5556.65");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3899.65,5653.49");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5517.51,5587.55");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5888.25,5597.94");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=183.09,5450.30");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1831.55,5390.16");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3089.14,5495.39");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4298.68,5300.25");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4736.65,5506.49");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5128.57,5313.37");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=851.45,5254.96");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1250.83,5210.36");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2223.91,5172.45");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2927.15,5096.18");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3627.19,5265.64");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4777.22,5088.67");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5604.04,5069.83");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5991.10,5098.77");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=983.23,4823.87");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2714.43,4784.05");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3441.22,4966.22");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3992.13,5003.29");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4419.06,4915.45");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=98.75,4660.88");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=463.58,4767.57");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1449.00,4593.06");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1868.57,4638.18");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3982.57,4642.18");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4808.32,4651.73");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5326.04,4704.03");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5922.24,4649.59");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=632.95,4423.37");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2420.81,4385.51");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3225.34,4319.70");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3588.68,4393.45");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4506.36,4419.36");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5258.35,4303.40");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5608.42,4485.14");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=119.92,4180.67");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1071.75,4076.32");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1466.44,4198.02");
    }

}

