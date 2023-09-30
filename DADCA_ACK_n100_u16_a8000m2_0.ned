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
        @display("i=misc/sensor;p=1577.88,7758.88");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1990.46,7824.21");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2361.31,7902.02");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2807.85,7776.05");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3489.15,7799.66");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3911.44,7765.31");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4419.59,7942.31");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4823.15,7827.40");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=448.35,7700.54");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=759.60,7523.09");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1198.55,7555.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3132.97,7638.47");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5120.33,7604.85");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5597.66,7727.64");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=164.95,7327.76");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2304.10,7311.23");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3527.48,7430.20");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4046.76,7409.59");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4568.40,7471.17");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=532.02,7254.04");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=885.07,7086.08");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1253.34,7058.92");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1956.70,7188.21");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2608.92,7063.88");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3226.73,7221.48");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3636.11,7079.65");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5138.63,7252.92");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5456.84,7062.62");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5867.76,7201.91");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=305.56,6916.07");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1673.51,6766.26");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4420.52,6890.07");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4862.55,6834.94");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1065.89,6710.31");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2358.65,6674.98");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3048.68,6725.42");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3646.79,6590.59");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4010.43,6601.80");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5528.47,6621.00");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5981.74,6604.23");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=25.15,6349.03");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1404.08,6426.33");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2823.81,6427.04");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3236.38,6406.12");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4356.46,6417.79");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5186.72,6296.98");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5735.46,6327.99");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=276.69,6058.42");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=691.63,6188.87");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1577.62,6103.47");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2045.12,6234.92");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4070.69,6191.97");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4676.61,6264.04");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2407.06,5938.78");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2989.48,5885.32");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3517.98,5995.81");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5422.58,5972.77");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5876.77,5947.04");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=35.02,5756.67");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=541.24,5683.46");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1194.11,5530.15");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2027.85,5647.29");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2449.99,5533.07");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3121.52,5530.91");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3796.64,5629.73");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4163.29,5708.00");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4866.68,5680.39");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5602.10,5655.80");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5965.49,5596.47");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=19.63,5375.15");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=772.03,5414.70");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1698.45,5310.31");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4483.98,5520.41");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5513.62,5286.28");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2102.20,5204.22");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2702.44,5081.49");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3398.11,5265.69");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4295.42,5184.32");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4724.61,5260.64");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5167.28,5135.28");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=330.88,4857.22");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=737.02,5002.11");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1293.70,4985.37");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2101.22,4828.38");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3122.62,4971.85");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3918.46,4879.05");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4867.75,4940.44");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5400.67,4855.79");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5860.66,4955.22");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=573.46,4554.79");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=946.61,4571.99");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1736.86,4546.06");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3166.83,4595.15");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4461.21,4749.78");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=210.22,4382.14");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2161.68,4364.20");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2629.50,4385.40");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3560.65,4450.59");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4233.10,4395.09");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5853.09,4288.87");
    }

}

