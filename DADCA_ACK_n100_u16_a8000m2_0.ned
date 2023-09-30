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

network DADCA_ACK_n100_u16_a8000m2_0
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
        @display("i=misc/sensor;p=1228.50,7785.23");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1731.85,7880.29");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2311.70,7771.39");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2716.19,7898.06");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4313.74,7943.62");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5997.31,7858.29");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3125.89,7705.34");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3533.35,7614.85");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4039.22,7712.27");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5063.76,7699.09");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5714.05,7615.46");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=816.83,7355.62");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1454.19,7313.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1851.83,7433.85");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2532.92,7429.01");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3135.03,7266.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4534.21,7377.94");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5335.55,7281.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=292.55,7090.70");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3697.42,7126.91");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4133.45,7165.81");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4961.88,7137.14");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5655.77,7081.95");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=871.28,6952.43");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2357.24,6880.15");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2691.71,6770.64");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3221.21,6765.48");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4541.34,6885.90");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5253.41,6890.98");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=315.04,6623.52");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=665.17,6655.01");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1174.49,6717.45");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1704.07,6744.72");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2286.17,6520.01");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3653.36,6643.97");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4026.85,6565.00");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1574.10,6344.55");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2933.19,6494.38");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3515.65,6274.36");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4280.89,6299.13");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4959.14,6470.38");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5327.87,6477.31");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=262.00,6126.54");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=697.96,6198.06");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1227.11,6216.29");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2630.67,6142.97");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4583.75,6089.42");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5162.42,6058.88");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5820.14,6194.00");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1640.56,5882.99");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2108.77,6000.66");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3176.16,6011.52");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3636.93,5890.03");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4052.42,6010.76");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5546.07,5783.17");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=892.70,5549.45");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1273.66,5765.13");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2962.42,5629.74");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3320.22,5659.87");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=73.42,5432.11");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=446.06,5440.53");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1664.71,5522.07");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2021.30,5468.77");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2377.52,5366.73");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3503.59,5312.67");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3924.85,5436.63");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4314.05,5389.51");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4748.93,5497.16");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5911.39,5438.45");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=568.27,5088.08");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1096.36,5211.17");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2905.73,5120.64");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4975.41,5193.88");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5545.76,5135.50");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1162.14,4866.12");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1942.66,4898.00");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3515.04,4944.11");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4093.43,5002.41");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4654.60,5014.84");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5939.66,5028.93");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=11.13,4731.80");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=990.13,4550.88");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1547.05,4741.99");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2310.23,4690.97");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4018.55,4560.97");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5191.88,4644.47");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=170.76,4375.78");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2818.86,4439.25");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3508.49,4412.01");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4758.66,4490.57");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5557.08,4477.45");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=711.76,4230.17");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1304.60,4269.41");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1911.99,4250.76");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3367.56,4063.89");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4225.02,4204.66");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4837.86,4112.68");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5916.90,4278.02");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=508.84,3880.81");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2450.00,3912.07");
    }

}

