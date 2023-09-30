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

network DADCA_ACK_n50_u16_a8000m2_2
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
        @display("i=misc/sensor;p=506.68,7759.28");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1315.50,7992.30");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1930.97,7950.04");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=130.72,7688.55");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1422.37,7652.75");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1922.37,7597.25");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2721.62,7526.93");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=634.89,7414.71");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2311.83,7386.19");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=111.76,7063.39");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1210.74,7133.71");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1673.22,7210.88");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4529.55,7087.02");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=643.60,6857.86");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2193.26,6824.46");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2734.90,6979.50");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4106.27,6953.34");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5843.10,6963.20");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1080.43,6534.30");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1496.54,6572.77");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3652.98,6744.80");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4014.96,6534.00");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4412.90,6551.25");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4878.16,6692.24");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1968.65,6402.26");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5496.20,6483.51");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5832.22,6309.79");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=708.58,6130.39");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1391.09,6072.88");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3575.43,6175.94");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4000.70,6061.01");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4363.11,6091.38");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4909.58,6142.97");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5366.78,6097.79");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=241.88,5802.43");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1041.70,5921.97");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2117.79,6012.15");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2858.13,5809.29");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5751.36,5872.23");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1585.14,5746.00");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2573.91,5561.45");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3428.94,5539.92");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4713.23,5676.30");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=778.27,5429.12");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1314.96,5490.02");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1894.10,5473.26");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3054.89,5297.27");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4059.06,5512.83");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5797.25,5525.06");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=262.57,5135.87");
    }

}

