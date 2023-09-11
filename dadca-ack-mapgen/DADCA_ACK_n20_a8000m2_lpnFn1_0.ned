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

network DADCA_ACK_n20_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor;p=181.81,120.70");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=260.22,123.55");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=221.07,235.92");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=307.29,239.76");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=219.87,337.87");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=140.05,295.91");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=120.46,211.07");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=175.57,418.62");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=241.53,470.27");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=219.41,530.08");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=137.30,502.43");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=144.51,578.48");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=44.61,498.70");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=81.91,431.90");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=308.51,600.41");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=225.91,590.68");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=381.32,578.83");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=423.60,647.91");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=380.75,723.75");
    }

}

