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

network DADCA_ACK_n100_u16_a8000m2_4
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
        @display("i=misc/sensor;p=938.39,7871.75");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1528.67,7956.78");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2265.56,7983.19");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2804.51,7960.16");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3271.74,7897.36");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4062.67,7812.47");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5358.01,7857.86");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1661.58,7576.06");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2181.05,7567.55");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2637.40,7601.97");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5039.10,7653.06");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5715.72,7606.51");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=290.82,7438.01");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=806.08,7357.15");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1145.51,7483.98");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3134.33,7362.92");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3672.97,7287.33");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4379.00,7409.35");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5407.62,7269.77");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=537.46,7030.40");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1513.35,7163.83");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2085.49,7193.58");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3348.34,7047.34");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4083.07,7154.78");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4969.02,7239.38");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2743.82,6950.02");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3791.90,6880.54");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4741.61,6847.08");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5391.90,6909.03");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5763.29,7001.84");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=363.12,6604.85");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1034.49,6729.39");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1757.16,6654.40");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2262.70,6673.41");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3489.53,6589.36");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4223.88,6671.42");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5655.65,6528.41");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=9.15,6311.45");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1070.00,6313.62");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2355.09,6288.55");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2631.04,6504.41");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3018.79,6510.32");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3999.30,6358.44");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4822.99,6300.19");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5285.66,6452.83");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=367.35,6024.96");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=680.68,6198.56");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1466.58,6188.10");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1805.62,6084.70");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5684.22,6132.39");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=916.94,5830.40");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1595.34,5803.66");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3118.60,5964.44");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3737.93,5922.64");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4155.80,5847.87");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5083.24,5980.16");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=215.79,5694.19");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2151.64,5760.85");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2668.30,5530.98");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3155.13,5538.67");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3498.30,5633.95");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4666.01,5756.02");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5166.48,5545.69");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5555.29,5528.01");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=729.87,5522.62");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1294.83,5437.46");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1788.32,5432.43");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2202.67,5338.66");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3848.68,5458.23");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5974.82,5319.53");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=295.52,5136.98");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=776.32,5146.15");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2872.25,5068.94");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3343.07,5074.03");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4213.32,5202.12");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4700.21,5138.48");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5382.34,5188.68");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1681.59,4886.64");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2201.81,4811.69");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3685.22,4997.90");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4215.93,4823.16");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4865.95,4821.03");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5285.88,4847.23");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5672.66,4876.70");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=418.76,4593.70");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=796.36,4741.61");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1288.59,4693.78");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3569.87,4581.48");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=137.04,4300.11");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=978.57,4376.83");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1738.86,4445.95");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2385.36,4294.93");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2884.50,4516.28");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4513.40,4381.74");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5096.74,4515.43");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5549.25,4344.26");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5913.60,4531.43");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=736.24,4094.36");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1462.50,4042.43");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2874.21,4149.87");
    }

}

