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

network DADCA_ACK_n20_a8000m2_lpnFn1_5
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
        @display("i=misc/sensor,p=37.87,80.61");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=44.12,151.20");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=80.44,201.11");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=155.39,166.70");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=194.80,101.57");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=227.80,183.92");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=201.70,238.92");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=312.51,260.82");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=320.62,195.85");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=376.28,237.08");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=388.93,335.16");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=294.82,345.39");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=237.28,363.05");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=179.21,427.88");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=128.33,394.74");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=71.96,315.49");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=316.52,404.56");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=241.37,490.04");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=271.34,569.87");
    }

}

