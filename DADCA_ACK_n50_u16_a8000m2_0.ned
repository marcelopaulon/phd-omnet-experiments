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

network DADCA_ACK_n50_u16_a8000m2_0
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
        @display("i=misc/sensor;p=169.44,7790.87");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2257.83,7948.77");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3176.46,7940.25");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3694.71,7874.22");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5313.95,7915.30");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5759.50,7778.37");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=633.86,7727.48");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1135.82,7573.75");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1631.94,7539.76");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2786.11,7740.75");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4083.55,7739.71");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4639.20,7746.76");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5509.81,7516.23");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=815.78,7407.16");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2207.99,7480.50");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2524.65,7264.36");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3479.11,7398.85");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4093.49,7339.40");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4954.25,7479.20");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=207.75,7162.34");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=808.66,7022.73");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1159.60,7152.20");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1571.73,7050.26");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3189.28,7106.07");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4557.45,7141.39");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=336.72,6798.71");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2113.73,6911.57");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3698.16,7000.42");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4002.31,6782.73");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4488.92,6791.32");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5000.79,6988.65");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5784.13,6967.18");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1215.88,6575.20");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1804.33,6571.94");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2276.11,6556.57");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2716.58,6680.44");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3356.00,6566.62");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4797.63,6605.32");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5462.60,6694.91");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=213.18,6283.05");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1484.38,6318.27");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4126.04,6413.99");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5064.47,6348.12");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5857.39,6316.46");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=686.70,6236.87");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2518.38,6080.54");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3078.44,6146.12");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4451.91,6247.02");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5484.71,6063.96");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=748.29,5859.67");
    }

}

