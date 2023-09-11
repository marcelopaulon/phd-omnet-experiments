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
        @display("i=misc/sensor,p=155.82,178.30");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=200.23,106.87");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=247.56,19.59");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=154.90,42.81");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=317.63,116.18");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=239.47,162.39");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=301.65,231.93");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=264.61,301.21");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=178.45,283.07");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=238.66,390.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=300.53,405.18");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=139.87,354.67");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=128.26,237.97");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=63.29,238.49");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=26.54,171.52");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=88.75,161.75");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=30.99,77.99");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=376.93,145.09");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=466.55,125.38");
    }

}

