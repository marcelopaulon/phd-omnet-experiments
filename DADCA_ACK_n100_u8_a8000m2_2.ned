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

network DADCA_ACK_n100_u8_a8000m2_2
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
        @display("i=misc/sensor;p=501.68,7908.64");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3775.29,7802.54");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4165.63,7763.01");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4921.18,7968.65");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5712.23,7831.76");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1632.53,7620.90");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=736.42,7426.30");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1202.13,7502.56");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2375.51,7360.49");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3222.20,7409.61");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3773.98,7440.58");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4201.76,7404.57");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4760.97,7502.60");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5292.39,7486.37");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5963.21,7316.12");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=308.18,7090.39");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=731.88,7037.15");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1441.92,7122.41");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2886.11,7242.28");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3334.59,7045.10");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4330.47,7035.01");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4944.95,7195.71");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1940.52,6870.74");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3049.92,6835.82");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3868.18,6895.77");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5395.19,6859.04");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=130.94,6669.11");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=615.54,6679.71");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1030.37,6563.07");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2659.99,6657.01");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4595.44,6594.21");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5023.36,6744.88");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5783.10,6683.45");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1580.93,6383.01");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2215.21,6319.66");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2971.89,6471.73");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3397.59,6304.37");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4067.58,6443.07");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5162.31,6360.45");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5687.61,6278.63");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=545.33,6038.75");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1385.40,6087.38");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2634.81,6242.28");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4497.89,6205.15");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=102.00,5872.23");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=914.55,5881.34");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1911.29,5829.10");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2367.85,5966.92");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3169.95,5941.89");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3780.81,5947.62");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5583.59,5841.46");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=388.84,5579.73");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2344.00,5532.31");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2839.08,5630.19");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3616.65,5628.57");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4308.78,5684.76");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4784.28,5760.90");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5959.55,5744.33");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1422.96,5512.61");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3187.56,5521.60");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3957.75,5428.44");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4724.60,5298.40");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5369.93,5453.16");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5805.23,5334.30");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=402.05,5158.61");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1083.58,5106.44");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2132.99,5060.74");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2781.24,5077.23");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3166.06,5167.50");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3551.34,5182.73");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4237.10,5126.61");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5074.82,5191.23");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1647.42,4994.71");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5605.79,5007.32");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=118.64,4620.35");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=815.23,4626.10");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1330.23,4609.99");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2365.82,4776.13");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2925.28,4729.13");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3668.28,4734.57");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3999.09,4544.71");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4360.12,4590.87");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5039.76,4664.99");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5718.58,4642.26");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1594.71,4321.43");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2583.59,4443.47");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3352.29,4312.55");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5426.07,4333.18");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=558.52,4190.97");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1149.01,4110.26");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2295.80,4193.50");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3034.54,4141.16");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3716.52,4231.35");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4554.72,4189.58");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5114.24,4164.81");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1791.09,3971.96");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3397.25,3910.68");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4104.45,4005.65");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4801.96,3828.73");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=457.21,3751.31");
    }

}

