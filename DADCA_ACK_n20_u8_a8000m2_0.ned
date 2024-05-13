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

network DADCA_ACK_n20_u8_a8000m2_0
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
        @display("i=misc/sensor;p=880.60,7869.64");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1413.01,7918.40");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2020.53,7780.77");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5032.73,7937.41");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5592.85,7767.70");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1005.42,7512.49");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2341.19,7554.78");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2947.96,7744.08");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3810.37,7639.53");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4322.53,7541.14");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=29.04,7289.16");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1505.86,7331.46");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2712.02,7291.16");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4069.92,7292.95");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4914.93,7477.00");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5843.51,7481.95");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=598.35,7179.64");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2189.43,7192.06");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3243.14,7138.70");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4710.42,7085.20");
    }

}

