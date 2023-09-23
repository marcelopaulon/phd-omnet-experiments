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

network DADCA_ACK_n100_u8_a8000m2_1
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
        @display("i=misc/sensor;p=704.59,7892.16");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1353.39,7772.10");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2278.66,7927.00");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2921.65,7913.14");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3366.36,7774.29");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3748.80,7949.28");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4732.56,7806.32");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=170.72,7639.80");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=999.49,7667.23");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2431.52,7602.72");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3743.24,7515.55");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5227.90,7739.40");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5560.51,7541.39");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=659.59,7459.54");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1876.84,7467.32");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2937.12,7348.27");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4058.78,7297.37");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4731.35,7437.74");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5103.27,7365.94");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=218.77,7236.50");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1280.75,7147.88");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1643.42,7139.40");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3423.28,7160.97");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5639.50,7138.63");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=112.15,6813.58");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=765.01,6808.05");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1750.39,6791.13");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2433.71,6843.69");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2815.20,6872.57");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4523.28,7008.95");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5287.05,6941.52");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=484.39,6579.68");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1155.37,6723.12");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3291.84,6604.04");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3733.71,6578.75");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4213.31,6740.42");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5190.64,6550.92");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=183.61,6283.36");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1342.21,6287.78");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2264.03,6332.82");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4285.14,6368.25");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5801.59,6394.23");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=580.42,6182.52");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1016.27,6098.61");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1641.97,6045.23");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2858.16,6258.41");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3405.13,6204.41");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3956.64,6237.73");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4876.07,6182.71");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=38.31,5834.40");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=447.96,5848.60");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2394.37,5902.21");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2768.32,5918.06");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3133.34,5832.21");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4262.27,5887.69");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4677.64,5892.28");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5312.96,6001.43");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5726.85,5908.95");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=966.53,5727.77");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1320.74,5715.53");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1771.23,5703.24");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2199.54,5563.02");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3746.59,5661.70");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4124.16,5548.21");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5087.73,5702.63");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=57.15,5427.75");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1511.56,5402.92");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1876.86,5335.50");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2680.68,5494.82");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3546.83,5314.02");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5745.87,5515.55");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=428.92,5270.89");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1000.44,5056.22");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2867.14,5095.09");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3190.04,5238.36");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4137.34,5150.27");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4566.42,5273.08");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5140.08,5151.59");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5917.64,5192.21");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=32.95,5011.17");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1488.82,5024.95");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2227.42,5029.44");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3340.04,4896.17");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4857.18,4848.47");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=504.42,4636.71");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1308.50,4551.79");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1728.28,4711.68");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2663.19,4772.84");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3029.22,4653.40");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4093.16,4581.67");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4526.41,4693.33");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5535.71,4677.28");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=601.45,4299.83");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=989.42,4332.40");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1624.92,4352.53");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2034.37,4489.69");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2447.71,4489.43");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3456.13,4474.25");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4878.28,4459.30");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5712.74,4334.01");
    }

}

