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

network DADCA_ACK_n100_u2_a8000m2_9
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
        @display("i=misc/sensor;p=208.02,7799.01");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1438.44,7754.31");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2557.68,7940.20");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3101.37,7817.76");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3592.43,7998.35");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3951.44,7779.92");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=457.79,7509.02");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1904.47,7694.16");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4793.95,7656.43");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5197.84,7511.42");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5672.09,7745.19");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1104.81,7286.37");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1968.57,7321.72");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2338.10,7326.92");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2685.67,7275.57");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3617.37,7311.39");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4169.00,7459.14");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3266.80,7132.73");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3994.71,7140.66");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4612.62,7204.12");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5596.00,7196.70");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=587.72,6992.22");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1013.69,6808.58");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1412.18,6995.61");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1885.23,6842.38");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2315.78,6892.21");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3523.46,6892.27");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4535.21,6772.47");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=720.03,6551.40");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1386.84,6570.93");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2391.74,6528.78");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2717.22,6693.77");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4029.15,6639.49");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5355.03,6641.15");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=15.67,6413.54");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3143.93,6330.54");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3484.32,6475.06");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4881.07,6323.30");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=727.37,6175.24");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1534.74,6036.87");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2050.39,6180.95");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2524.68,6083.67");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4128.00,6203.52");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4652.26,6020.83");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5283.18,6219.34");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5934.94,6256.91");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=434.45,5981.66");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1095.02,5792.43");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2771.40,5772.72");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3242.29,5873.68");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3645.93,5888.38");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5708.26,5888.29");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=212.44,5615.50");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1572.80,5682.95");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2148.54,5687.99");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4331.38,5654.33");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4971.73,5653.36");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=620.54,5500.58");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1832.13,5411.39");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2890.71,5390.54");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4591.76,5291.15");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5405.65,5361.67");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5878.36,5334.04");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=594.68,5115.66");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1279.01,5141.31");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2245.67,5083.20");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2645.92,5111.56");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3542.84,5258.65");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3964.88,5165.91");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5019.65,5199.42");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=250.88,5022.11");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1654.90,4946.73");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3159.87,4881.91");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4543.15,4889.73");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5229.06,4827.29");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5778.01,4786.33");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=248.74,4666.78");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=863.62,4752.73");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1318.82,4686.27");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1781.77,4596.93");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2467.32,4701.90");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2822.22,4766.48");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4172.59,4770.10");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=583.86,4455.49");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=925.95,4377.65");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1365.46,4324.72");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2234.90,4321.50");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3228.30,4504.32");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3768.11,4532.34");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4954.05,4511.05");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5779.37,4331.62");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=55.30,4048.77");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1864.58,4139.14");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2791.24,4110.05");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3544.36,4090.18");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3963.22,4099.21");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4497.84,4223.58");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5073.54,4079.81");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=781.03,3979.71");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2408.70,3932.34");
    }

}

