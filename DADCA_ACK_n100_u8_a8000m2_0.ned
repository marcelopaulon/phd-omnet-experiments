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

network DADCA_ACK_n100_u8_a8000m2_0
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
        @display("i=misc/sensor;p=340.72,7866.94");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=962.74,7929.88");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1366.01,7786.37");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1671.94,7982.70");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2104.45,7988.51");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2377.58,7768.61");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2702.86,7945.08");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5328.45,7965.62");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1866.94,7638.47");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2995.85,7697.04");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3471.47,7630.69");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4607.65,7605.74");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5092.27,7570.73");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5713.78,7639.50");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=197.17,7350.77");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=847.33,7478.83");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1482.23,7396.92");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4760.94,7265.35");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1997.97,7133.50");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2351.29,7141.02");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2828.57,7150.55");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3269.38,7172.14");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3795.69,7060.77");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4210.49,7221.95");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5386.12,7065.67");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=47.56,6940.16");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=641.01,6905.73");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1257.43,6968.15");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4236.76,6820.58");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4735.75,6802.61");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5704.15,6888.71");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=882.34,6539.59");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1746.84,6541.75");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2225.49,6554.51");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2836.61,6680.98");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3382.16,6574.14");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3857.72,6675.12");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5496.42,6556.26");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5855.20,6552.76");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=42.88,6451.21");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=494.68,6465.38");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2644.16,6376.01");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3871.89,6287.69");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4446.00,6360.50");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5071.16,6470.44");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=348.86,6030.54");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1095.87,6059.32");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1482.42,6080.62");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2314.44,6169.94");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2887.50,6111.85");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3339.68,6214.59");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5469.16,6194.51");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5957.17,6101.95");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=726.61,5865.38");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1805.30,5864.00");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2544.44,5826.53");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3886.94,5831.94");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4437.09,5915.25");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4864.32,5891.90");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=93.35,5556.94");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1151.19,5697.04");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3289.15,5632.59");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5464.88,5605.47");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=404.06,5392.14");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1498.61,5429.15");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2075.55,5511.22");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2484.36,5374.78");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3778.47,5349.13");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4240.62,5490.06");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4613.00,5481.85");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5136.86,5464.75");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5923.63,5459.32");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=885.99,5191.59");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1371.12,5035.41");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1868.46,5225.50");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2214.23,5086.02");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3040.15,5171.31");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3427.07,5125.63");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4711.23,5122.47");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5467.06,5159.11");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=337.81,4815.34");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3805.62,4859.06");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5087.96,4824.52");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5771.26,4812.57");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1188.47,4709.98");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1594.91,4721.22");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2040.51,4642.24");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2526.10,4718.94");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2874.84,4766.82");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3273.78,4694.74");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=21.40,4427.36");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=727.61,4531.70");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1384.70,4398.50");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1877.32,4289.39");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2335.06,4414.60");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4010.63,4456.67");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5126.08,4404.31");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5548.63,4307.56");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=723.41,4123.46");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2825.03,4287.38");
    }

}

