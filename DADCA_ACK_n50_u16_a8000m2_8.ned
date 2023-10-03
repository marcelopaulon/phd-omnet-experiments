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

network DADCA_ACK_n50_u16_a8000m2_8
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
        @display("i=misc/sensor;p=6.80,7804.49");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=748.75,7917.35");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1095.26,7861.68");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2755.40,7910.38");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4192.40,7907.40");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=678.45,7552.63");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1481.04,7752.31");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1843.09,7707.17");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2411.84,7521.09");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2940.35,7571.90");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3798.55,7696.65");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4621.56,7712.56");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5076.78,7588.56");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5461.48,7744.41");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1174.00,7420.51");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1766.70,7298.71");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3983.29,7300.76");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4396.43,7413.77");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5692.06,7445.00");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=378.94,7254.39");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2192.09,7127.64");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2748.99,7192.23");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3097.52,7096.46");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3489.77,7246.58");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4751.77,7078.97");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5078.83,7234.85");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=9.95,6926.88");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=502.82,6763.73");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2727.31,6828.95");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3887.39,6941.58");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5506.29,6970.84");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=962.41,6748.88");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1532.60,6747.57");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2056.70,6760.71");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3109.54,6642.21");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3589.70,6723.86");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4255.09,6671.67");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5015.78,6615.26");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1721.20,6326.09");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2089.96,6368.76");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5554.21,6440.80");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=358.36,6196.64");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=743.22,6199.78");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2854.11,6207.07");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3243.65,6102.56");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3849.22,6239.04");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4598.91,6164.14");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5167.35,6266.34");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5867.22,6258.68");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=126.18,5823.49");
    }

}

