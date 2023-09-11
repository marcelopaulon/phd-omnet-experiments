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

network DADCA_ACK_n20_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor,p=199.21,96.72");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=223.78,14.38");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=308.73,9.57");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=353.54,61.25");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=276.88,110.67");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=437.69,22.06");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=506.80,73.17");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=559.70,37.83");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=648.80,38.84");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=656.80,116.46");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=710.16,44.72");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=795.60,20.93");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=765.76,85.28");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=855.20,87.90");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=832.44,153.88");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=869.73,4.67");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=923.08,160.40");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=753.89,172.84");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=985.38,167.71");
    }

}

