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

network DADCA_ACK_n20_a8000m2_lpnFn1_6
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=73.15,156.18");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=151.61,215.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=220.85,257.48");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=141.63,303.14");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=275.78,230.65");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=299.72,326.59");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=306.87,387.48");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=223.21,406.56");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=169.41,463.05");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=215.91,323.05");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=111.82,370.74");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=22.68,364.73");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=59.14,305.72");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=44.77,448.24");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=52.99,220.12");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=126.26,13.57");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=176.93,90.70");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=16.06,19.83");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=242.28,134.20");
    }

}

