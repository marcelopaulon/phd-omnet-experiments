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

network DADCA_ACK_n100_a8000m2_7
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
        @display("i=misc/sensor;p=447.36,7981.02");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=512.24,7995.55");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=571.61,7981.06");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=633.42,7976.16");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=705.42,7972.06");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=37.01,7936.61");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=173.10,7925.96");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=492.15,7930.37");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=90.33,7904.09");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=303.86,7903.39");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=424.08,7891.41");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=552.41,7905.61");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=686.84,7878.66");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=19.90,7864.65");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=114.08,7833.63");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=193.59,7824.58");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=288.75,7803.44");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=400.32,7790.29");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=463.96,7819.40");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=531.19,7824.22");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=735.12,7817.47");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=837.48,7825.93");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=77.18,7784.03");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=155.74,7754.62");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=217.16,7744.64");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=562.08,7769.86");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=631.18,7780.24");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=757.58,7752.95");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=874.02,7759.63");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=947.78,7759.34");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=293.07,7724.07");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=482.60,7704.25");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=584.26,7702.69");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=413.66,7671.93");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=685.70,7671.83");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=749.76,7659.49");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=821.71,7660.37");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=930.94,7675.62");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=996.47,7681.83");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=184.94,7638.85");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=251.07,7635.75");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=503.16,7616.45");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=883.11,7615.51");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=114.43,7599.89");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=338.03,7581.61");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=579.47,7600.26");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=645.17,7573.96");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=711.42,7580.38");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=995.89,7602.71");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=65.71,7534.61");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=269.06,7532.84");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=459.64,7566.54");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=790.81,7545.41");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=947.26,7535.42");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=182.83,7522.50");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=399.88,7521.99");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=515.76,7486.98");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=603.73,7490.85");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=720.44,7489.51");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=839.41,7501.95");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=91.81,7470.16");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=172.76,7457.52");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=296.20,7474.96");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=40.71,7427.09");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=234.93,7400.14");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=401.88,7415.73");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=479.52,7397.98");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=548.71,7431.74");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=753.74,7410.50");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=818.16,7437.40");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=910.26,7420.52");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=976.84,7417.34");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=173.87,7371.39");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=296.53,7394.56");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=639.95,7380.01");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=703.75,7357.49");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=72.17,7342.21");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=169.03,7310.06");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=269.56,7318.88");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=380.64,7312.94");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=486.75,7331.57");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=769.09,7344.59");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=833.31,7334.24");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=927.98,7351.31");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=993.81,7321.76");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=22.93,7294.98");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=117.06,7272.93");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=595.87,7295.93");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=660.61,7267.10");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=780.74,7269.60");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=852.27,7274.12");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=176.83,7242.19");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=277.04,7242.02");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=425.42,7257.35");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=534.61,7264.59");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=718.82,7244.68");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=911.63,7260.07");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=366.79,7222.31");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=554.36,7193.29");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=757.09,7194.45");
    }

}

