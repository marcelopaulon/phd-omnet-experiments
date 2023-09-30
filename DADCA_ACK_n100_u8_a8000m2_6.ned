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

network DADCA_ACK_n100_u8_a8000m2_6
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
        @display("i=misc/sensor;p=496.41,7915.13");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1275.79,7816.99");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1979.35,7935.85");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2484.10,7835.90");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3847.47,7926.44");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4195.49,7853.68");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=147.94,7746.81");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1671.15,7604.96");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2918.15,7665.66");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3338.22,7606.96");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3833.74,7546.50");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4650.86,7732.77");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5284.08,7521.61");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=290.07,7423.01");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=729.61,7428.18");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2219.42,7368.93");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4149.58,7385.26");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=597.95,7088.85");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1364.65,7154.97");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1739.89,7031.01");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2741.66,7150.90");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3158.86,7109.00");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4861.72,7252.18");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1087.44,6914.29");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2107.82,6837.83");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3494.22,6998.52");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4152.62,6997.37");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4524.61,6948.65");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5222.82,6898.18");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=38.23,6655.70");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=476.08,6668.58");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1569.64,6666.66");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2674.01,6605.81");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3026.95,6704.75");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3871.12,6739.33");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4793.17,6592.40");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5805.57,6703.65");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=387.68,6283.33");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=982.22,6461.69");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1865.66,6423.16");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4343.44,6506.13");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5438.49,6430.94");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=911.02,6075.64");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1423.04,6029.32");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2177.07,6153.08");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3013.47,6262.23");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3404.69,6172.94");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4016.61,6179.72");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4555.72,6203.98");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4915.80,6104.43");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5335.46,6034.31");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=75.23,5964.07");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2743.27,5983.03");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5719.49,5818.87");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=729.31,5752.68");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1141.30,5691.10");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1798.98,5729.71");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2259.03,5648.07");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3158.47,5737.74");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3738.16,5662.48");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4315.30,5625.37");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5251.24,5610.08");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=289.27,5518.19");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1361.28,5408.34");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1712.95,5352.06");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2650.67,5490.67");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4788.63,5513.82");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=105.66,5138.21");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=821.39,5143.89");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2297.73,5164.67");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3467.22,5164.10");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3929.10,5204.42");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4391.69,5187.55");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4913.31,5054.59");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5570.57,5188.15");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2095.00,4857.82");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2949.36,5008.18");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4596.29,4885.05");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5835.26,4886.69");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=124.10,4563.29");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=554.69,4693.01");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1065.22,4611.11");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1424.13,4736.66");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2498.88,4609.27");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3412.99,4567.84");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4020.32,4707.19");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5479.29,4751.00");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=399.86,4323.79");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=762.24,4288.83");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1650.51,4461.84");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2027.26,4507.93");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2754.50,4346.41");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4669.54,4527.21");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5324.85,4311.17");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5999.38,4375.04");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1402.86,4111.55");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1956.39,4138.26");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3672.11,4192.93");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4462.72,4240.83");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=516.65,3859.52");
    }

}

