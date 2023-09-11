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

network DADCA_ACK_n20_a8000m2_lpnFn1_2
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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=39.41,154.52");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=92.11,230.89");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=113.04,293.79");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=28.93,345.78");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=97.13,356.35");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=226.28,268.44");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=259.67,217.24");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=297.11,295.84");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=265.33,122.36");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=264.45,38.05");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=334.27,2.56");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=394.05,54.01");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=405.81,121.61");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=443.21,191.52");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=432.50,271.24");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=515.83,306.84");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=484.93,237.00");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=478.66,142.32");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=378.67,175.23");
    }

}

