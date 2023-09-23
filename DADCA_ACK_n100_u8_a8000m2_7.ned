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

network DADCA_ACK_n100_u8_a8000m2_7
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
        @display("i=misc/sensor;p=2775.24,7892.94");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3347.19,7816.29");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4586.96,7997.91");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1852.85,7748.57");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2271.62,7586.93");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2861.66,7548.98");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3943.50,7572.20");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4429.90,7530.77");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5727.50,7695.75");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1839.53,7358.55");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3278.46,7347.51");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4806.39,7388.13");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5191.79,7370.65");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1507.37,7190.55");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2575.27,7257.02");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3054.31,7044.13");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3543.54,7063.54");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3915.29,7067.46");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4461.26,7101.66");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5107.61,7014.81");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5562.22,7127.47");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=301.10,6996.27");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1009.36,6771.24");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1420.94,6814.57");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2148.73,7009.67");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1958.46,6670.39");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3384.03,6667.70");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4903.73,6569.68");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5843.89,6585.82");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=696.69,6489.42");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1033.08,6319.33");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2680.25,6484.18");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4461.13,6382.67");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5364.09,6501.80");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=332.46,6257.10");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1608.42,6243.90");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2215.61,6205.65");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3308.61,6163.12");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3965.64,6246.69");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4940.27,6115.13");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5618.70,6193.63");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=637.31,5822.67");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1338.61,5922.68");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2532.47,5793.05");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5217.87,5793.92");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5836.01,5805.33");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1870.59,5753.57");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2984.27,5613.70");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4162.24,5728.04");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=234.91,5410.25");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1065.67,5302.08");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1378.88,5516.32");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2141.00,5305.10");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3595.17,5499.12");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3924.38,5338.49");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5594.30,5361.19");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=671.78,5169.35");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1382.35,5063.57");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2609.77,5150.04");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3145.82,5222.20");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4283.47,5104.09");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4882.95,5170.37");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5243.75,5123.02");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=169.96,5015.07");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1016.87,4798.91");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3301.92,4830.51");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3699.12,5001.26");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5863.08,4976.02");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=209.76,4623.39");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2239.41,4754.75");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2863.71,4714.09");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4026.31,4700.45");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4497.02,4546.02");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4861.63,4692.88");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5337.57,4681.79");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5696.48,4594.50");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=590.18,4511.19");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1065.54,4407.57");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1549.01,4525.40");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3535.58,4369.42");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=266.77,4216.25");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1600.68,4172.63");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2115.02,4171.18");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2690.05,4261.33");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3154.97,4161.58");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3998.61,4181.00");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4485.35,4104.27");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5184.06,4217.73");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5545.36,4065.10");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=749.92,3944.96");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1193.60,3913.72");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1614.07,3810.64");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2333.25,3704.60");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2927.31,3725.14");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3823.98,3769.35");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5229.33,3720.98");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=190.86,3477.47");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1200.49,3353.44");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2080.97,3448.13");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3190.45,3455.80");
    }

}

