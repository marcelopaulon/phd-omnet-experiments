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

network DADCA_ACK_n20_a8000m2_0
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
        @display("i=misc/sensor;p=382.05,7951.82");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1000.27,7839.24");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3109.90,7765.73");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3679.42,7741.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4689.39,7795.18");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5166.25,7875.83");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5719.54,7932.10");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=275.60,7524.78");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1562.38,7631.07");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2020.82,7717.69");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2551.06,7565.31");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4182.44,7658.75");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5456.26,7644.39");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=624.10,7353.25");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1084.24,7349.58");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1796.55,7321.91");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2258.21,7323.99");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2950.12,7359.50");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4340.93,7251.21");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4786.43,7400.65");
    }

}

