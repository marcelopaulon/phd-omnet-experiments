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

network DADCA_ACK_n20_a8000m2_7
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
        @display("i=misc/sensor;p=570.50,7744.36");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1242.71,7898.45");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1772.85,7958.70");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2887.28,7813.33");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3465.05,7936.64");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4101.99,7852.09");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4811.87,7859.07");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5548.36,7856.75");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=130.82,7726.78");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1749.38,7537.08");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2532.89,7697.92");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3655.49,7539.96");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4556.01,7483.62");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=944.97,7429.13");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1357.46,7256.12");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2576.67,7288.46");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3026.27,7414.02");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3376.64,7297.12");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5068.86,7287.87");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5564.56,7472.03");
    }

}

