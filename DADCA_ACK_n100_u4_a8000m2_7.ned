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

network DADCA_ACK_n100_u4_a8000m2_7
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
        @display("i=misc/sensor;p=244.18,7906.20");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=909.99,7934.04");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1453.04,7998.16");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1931.20,7880.54");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2320.57,7803.18");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4051.25,7892.32");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5094.11,7915.42");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5519.80,7947.48");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1591.37,7627.85");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5762.62,7586.60");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=342.86,7491.51");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=993.22,7447.30");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2289.54,7338.73");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3146.29,7503.62");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3697.67,7425.68");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4311.74,7496.37");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5299.08,7490.86");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1853.59,7134.83");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4011.38,7087.38");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4621.19,7175.36");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5553.01,7119.76");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5930.19,7041.92");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=744.95,6943.45");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1378.55,6886.12");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2749.62,6959.24");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3573.81,6943.53");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=907.86,6589.99");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2021.62,6656.08");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3027.99,6550.50");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4893.10,6694.64");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5690.15,6597.00");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1365.48,6486.90");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2384.87,6414.99");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3903.64,6413.23");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5195.66,6456.87");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=329.34,6038.39");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1772.11,6258.25");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2366.03,6062.88");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2764.26,6240.27");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3300.31,6107.72");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4383.18,6161.64");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5003.07,6102.99");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5533.23,6032.05");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1118.62,5833.90");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1734.94,5829.39");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2841.55,5801.23");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3835.49,5865.77");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4739.24,5819.33");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5969.07,5959.13");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=13.96,5772.49");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2100.74,5659.02");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2489.76,5648.57");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3494.29,5620.76");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5056.34,5580.55");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=113.27,5326.64");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=717.15,5457.12");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1336.00,5432.06");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2104.16,5300.66");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4034.82,5298.96");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5632.03,5340.77");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1704.09,5255.16");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2760.96,5234.37");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3220.60,5257.11");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4414.41,5273.19");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5967.68,5225.32");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=709.90,4861.53");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1136.54,4929.85");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3008.43,4844.13");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3776.61,4985.40");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5117.48,4978.78");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=128.82,4745.17");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1557.18,4689.97");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2353.86,4739.30");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3456.54,4696.47");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3953.54,4663.69");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4368.33,4585.94");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5720.89,4632.04");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=71.89,4308.75");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=519.79,4512.82");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1325.10,4401.35");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2075.08,4402.53");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2641.00,4385.81");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3629.08,4347.73");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5017.24,4292.37");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=979.09,4258.83");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1615.03,4164.22");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2368.78,4150.32");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3076.67,4181.02");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4467.89,4131.93");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5434.91,4233.25");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5975.50,4199.10");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1286.79,3868.34");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3231.43,3818.21");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4012.83,3982.61");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5201.25,3935.33");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5830.84,3824.68");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=345.76,3757.43");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=887.16,3787.18");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1942.38,3753.29");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2496.55,3614.70");
    }

}

