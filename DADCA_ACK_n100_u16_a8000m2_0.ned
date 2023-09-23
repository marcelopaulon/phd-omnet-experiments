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
        @display("i=misc/sensor;p=2499.16,7813.97");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3903.13,7802.04");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4551.13,7907.37");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5548.83,7944.84");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5988.14,7852.07");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2091.73,7541.27");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3250.25,7710.73");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3635.92,7572.06");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5112.07,7666.82");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5608.72,7537.00");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2410.11,7331.54");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2990.63,7446.86");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4688.67,7412.69");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5275.46,7327.19");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1747.63,7213.97");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3591.23,7221.78");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4255.84,7252.29");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=350.69,6791.24");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1402.24,6876.90");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2141.12,6894.04");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2656.69,6958.39");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4085.67,6810.15");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4688.98,6820.19");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5046.81,6869.01");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5560.68,6926.67");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5914.57,6959.03");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=64.69,6533.91");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=927.17,6678.91");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2497.87,6607.66");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3191.17,6598.80");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3654.88,6670.85");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5378.36,6606.26");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5728.84,6544.96");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=697.05,6295.92");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1182.67,6344.21");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1755.94,6437.01");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2186.36,6276.34");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2857.99,6395.03");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3937.08,6317.43");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4508.20,6476.76");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5028.65,6274.78");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1454.48,6097.25");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3406.04,6231.79");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4381.16,6071.25");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5585.59,6172.68");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=19.27,5917.72");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=513.11,5857.46");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1855.71,5964.08");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2364.28,5906.73");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3007.81,6013.68");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4714.86,5807.46");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5952.78,5958.00");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1307.20,5561.79");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1831.67,5560.68");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2409.41,5529.11");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3782.61,5722.98");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4279.38,5677.51");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4997.21,5591.46");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5317.75,5754.21");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=284.12,5352.12");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=723.34,5513.44");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2869.54,5475.34");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3378.24,5283.57");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4573.07,5378.59");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5944.88,5513.18");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1266.69,5210.85");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1985.03,5176.96");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2508.43,5169.99");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3863.34,5146.51");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5397.05,5169.04");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=71.60,4945.04");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=756.23,5021.47");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1416.48,4885.27");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2909.22,5028.84");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3273.95,4866.63");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4112.79,4871.90");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4817.37,4954.20");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5873.78,4947.46");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=824.95,4678.25");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1162.73,4542.99");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2207.67,4641.37");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2695.98,4749.79");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3714.75,4731.58");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4383.91,4538.80");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5497.82,4558.10");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=195.96,4463.14");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1927.80,4326.70");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3634.45,4365.47");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4859.10,4471.23");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=757.92,4219.62");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1522.78,4235.93");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2562.54,4241.18");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3039.40,4206.68");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5144.97,4249.36");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5605.20,4162.86");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=415.80,4014.32");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=949.87,3899.54");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1784.98,3936.24");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3568.58,3876.82");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4169.23,4035.41");
    }

}

