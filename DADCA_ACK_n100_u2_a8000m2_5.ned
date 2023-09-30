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

network DADCA_ACK_n100_u2_a8000m2_5
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
        @display("i=misc/sensor;p=107.02,7829.47");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1566.17,7968.67");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2322.19,7781.12");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2790.91,7787.97");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3112.83,7973.30");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3482.18,7894.05");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5813.83,7879.14");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=713.93,7691.49");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1091.43,7635.96");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3277.93,7574.89");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4130.92,7702.84");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4800.40,7648.83");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=273.21,7270.84");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2274.88,7384.34");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3789.54,7331.44");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5239.25,7294.68");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5623.53,7500.08");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2672.47,7079.21");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3346.84,7187.97");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4454.84,7137.74");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4851.83,7076.93");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5555.48,7087.05");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=875.79,6975.21");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2154.91,6870.33");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3039.65,6922.15");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3791.15,6856.08");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4142.57,6772.29");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5933.43,6879.09");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=280.04,6617.92");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1391.74,6524.48");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2949.36,6566.61");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3492.21,6662.81");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5082.13,6727.01");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5551.37,6655.51");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=953.53,6318.94");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2041.72,6271.84");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2611.51,6465.40");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3688.72,6308.10");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4603.47,6507.00");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5190.60,6297.39");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5859.82,6324.83");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=346.15,6253.17");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1480.34,6147.72");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4351.19,6143.28");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=887.57,5858.34");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2485.30,5869.84");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3208.96,5929.47");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3753.45,5915.18");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4876.15,6009.04");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5438.45,5897.42");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=179.46,5763.02");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1144.22,5618.04");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1581.60,5750.31");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2006.60,5730.55");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2731.26,5546.44");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3399.43,5563.32");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4043.71,5555.06");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5137.85,5718.14");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5703.79,5654.19");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=253.47,5357.77");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=656.01,5426.10");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2154.95,5402.44");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4572.69,5508.08");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5067.74,5309.91");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=768.06,5086.55");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1491.07,5179.58");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2325.62,5088.44");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2976.76,5059.79");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3453.65,5204.83");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3783.48,5087.65");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5832.88,5212.12");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1775.25,4858.05");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4182.58,4852.97");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4770.65,4913.87");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5367.60,4992.50");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=485.72,4724.79");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=925.71,4681.12");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1307.70,4708.43");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2231.73,4665.40");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3443.06,4619.01");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3795.82,4713.21");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5350.45,4586.73");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=479.63,4365.64");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1080.74,4305.83");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1811.05,4500.19");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2644.67,4479.26");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3006.28,4491.67");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4095.54,4348.46");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5007.30,4328.36");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2186.78,4099.35");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3336.01,4217.21");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4454.45,4257.55");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=363.29,3820.46");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1609.42,3839.53");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2535.62,3894.29");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3038.01,3931.05");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3495.98,3852.69");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3843.11,3925.95");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4894.03,3983.94");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5271.73,3945.63");
    }

}

