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

network DADCA_ACK_n50_u8_a8000m2_1
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
        @display("i=misc/sensor;p=1161.21,7958.38");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2436.03,7805.48");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3124.05,7795.92");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3494.71,7944.94");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3991.01,7943.78");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4884.89,7915.59");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5260.04,7817.50");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5956.80,7998.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=40.73,7697.23");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=532.14,7695.41");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1574.20,7621.05");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2033.61,7582.82");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2841.52,7568.92");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3849.31,7508.20");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4544.76,7528.72");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5448.61,7518.02");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=895.35,7459.50");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4970.66,7289.72");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5932.84,7413.08");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=246.82,7245.03");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=646.88,7192.99");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1147.21,7118.00");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2040.60,7186.32");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3394.81,7180.35");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4335.98,7238.32");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5939.13,7061.79");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1034.16,6785.26");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1504.48,7004.05");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2801.67,6865.20");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3737.00,6882.85");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4331.22,6791.23");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4754.96,6897.08");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5119.28,6945.36");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5546.58,6924.44");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=665.46,6517.68");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1505.57,6581.80");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1992.67,6643.03");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4950.18,6524.87");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2547.63,6418.73");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3199.69,6435.03");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3603.31,6481.44");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3980.95,6508.94");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5301.41,6275.33");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5621.48,6442.13");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=74.97,6027.99");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=562.59,6179.88");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1055.99,6093.15");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1648.81,6248.23");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2077.32,6204.81");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3692.22,6046.13");
    }

}

