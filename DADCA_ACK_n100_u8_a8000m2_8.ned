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

network DADCA_ACK_n100_u8_a8000m2_8
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
        @display("i=misc/sensor;p=367.94,7802.40");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=926.54,7850.50");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2280.33,7977.59");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5068.40,7913.61");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1140.91,7513.34");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1611.93,7629.06");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2043.16,7593.43");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2520.20,7584.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3037.73,7702.63");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3918.35,7743.09");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4395.97,7620.65");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5605.68,7751.89");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=277.39,7311.20");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3571.26,7438.78");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4083.02,7341.70");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4859.06,7323.68");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5478.20,7369.11");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=706.31,7225.63");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1403.97,7066.96");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1892.41,7256.61");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2377.10,7213.72");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3171.20,7186.22");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4419.12,7189.82");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5989.98,7235.94");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2758.66,6906.83");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3634.23,6892.97");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4877.73,6893.70");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5567.13,6966.92");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5903.60,6786.85");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=490.79,6707.72");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1047.86,6678.31");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1760.16,6623.61");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2315.22,6539.56");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3110.56,6708.68");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4000.16,6683.17");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4395.82,6733.11");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1018.79,6280.04");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2858.79,6427.41");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3555.21,6476.58");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4703.69,6411.12");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5276.19,6391.35");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=613.75,6202.95");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1686.94,6207.56");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2585.01,6170.33");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3174.12,6030.38");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3851.56,6265.96");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4215.33,6116.25");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5777.15,6233.05");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=162.58,5985.84");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1999.89,5921.60");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4663.11,5997.48");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=589.06,5706.35");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=971.63,5559.69");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1497.32,5633.71");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2058.80,5560.74");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2628.04,5772.01");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2980.78,5553.08");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4168.48,5612.85");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5291.01,5769.73");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=118.17,5319.90");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3344.69,5476.08");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4652.72,5369.73");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5748.10,5292.82");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=736.89,5064.01");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1896.11,5080.22");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2368.01,5094.56");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2711.45,5205.29");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3793.86,5109.21");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5224.70,5241.15");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=324.50,4913.37");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1176.27,4947.95");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3335.98,4998.38");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4446.50,4985.47");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4956.68,5005.42");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5913.03,4841.52");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=646.86,4663.69");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2066.87,4594.05");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2465.21,4668.32");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2903.54,4716.79");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5368.58,4779.88");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=151.73,4329.35");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=910.01,4377.68");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1293.97,4319.29");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1595.13,4520.20");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3284.67,4299.65");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3690.02,4528.58");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4047.56,4463.08");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4891.57,4462.72");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5591.68,4404.09");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=451.71,4076.26");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1946.89,4153.49");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2369.76,4190.18");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2738.55,4163.74");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3525.97,4044.94");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3974.41,4062.29");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4374.15,4207.48");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5344.07,4134.88");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5844.23,4103.87");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4659.20,3802.84");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5067.06,3882.25");
    }

}

