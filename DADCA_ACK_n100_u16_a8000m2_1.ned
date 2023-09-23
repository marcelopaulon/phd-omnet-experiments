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

network DADCA_ACK_n100_u16_a8000m2_1
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
        @display("i=misc/sensor;p=999.85,7901.55");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2300.96,7824.99");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4093.42,7867.94");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4650.76,7777.62");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5673.23,7753.36");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=871.47,7546.65");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1226.51,7507.52");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1560.49,7752.30");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2887.40,7678.20");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3364.66,7717.93");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5062.43,7515.63");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1870.27,7320.09");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3167.43,7360.77");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4109.46,7403.38");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4462.97,7337.13");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=284.32,7227.89");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=847.78,7059.95");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1333.34,7078.36");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2397.44,7202.18");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3740.27,7246.93");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5097.97,7045.81");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5710.15,7073.58");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1963.86,6782.93");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2377.20,6835.59");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2778.57,6847.86");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4386.71,6986.23");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1116.61,6617.07");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1502.65,6558.60");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3461.09,6733.84");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3963.95,6651.97");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4682.37,6762.26");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=206.34,6408.95");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2015.80,6425.66");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2985.68,6355.27");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4329.31,6288.12");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4737.16,6415.29");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5289.55,6378.09");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5777.81,6427.13");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=502.65,6103.28");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=863.49,6212.76");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1202.41,6123.61");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2445.82,6123.03");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3455.88,6130.74");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3875.69,6031.84");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5469.22,6053.59");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5841.98,6056.50");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=233.04,5812.62");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1672.17,5897.43");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2739.93,5822.66");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4818.49,5881.83");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1235.47,5690.76");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2395.72,5628.24");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3114.15,5760.81");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3538.70,5648.97");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3961.30,5526.47");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4487.28,5710.50");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5929.16,5676.20");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=779.69,5513.36");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1846.60,5423.26");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2589.03,5325.16");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3245.34,5298.53");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5297.18,5513.50");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5640.96,5422.38");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=329.62,5209.51");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1147.36,5165.21");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2161.41,5048.49");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2792.22,5034.63");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4495.80,5120.73");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5190.20,5053.26");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5713.51,5069.62");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=849.25,4871.51");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3162.24,4808.74");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3514.39,5009.74");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4065.47,4899.48");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=266.09,4665.97");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1413.25,4670.06");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2002.29,4651.20");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2494.09,4651.67");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4733.05,4756.31");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=765.91,4497.02");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3203.85,4299.28");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3523.67,4478.11");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3930.97,4461.28");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4417.09,4435.24");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5168.95,4328.72");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5693.06,4491.37");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=50.02,4140.87");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1235.59,4176.69");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1738.70,4125.41");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2169.13,4173.12");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2845.02,4263.64");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4444.43,4066.65");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4829.55,4172.86");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5939.81,4185.94");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=475.82,4031.32");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1055.42,3869.41");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4855.64,3813.22");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5171.37,3966.45");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5522.34,3911.27");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=235.15,3635.28");
    }

}

