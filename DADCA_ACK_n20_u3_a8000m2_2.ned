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

network DADCA_ACK_n20_u3_a8000m2_2
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
        @display("i=misc/sensor;p=8.01,7806.95");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2497.16,7889.21");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2963.83,7934.93");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=355.09,7693.90");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=750.24,7569.99");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1411.62,7651.38");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3299.56,7718.19");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3719.43,7638.89");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4084.78,7715.87");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4701.44,7530.27");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=169.04,7370.75");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1902.36,7490.28");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2455.38,7460.40");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2828.24,7374.57");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3251.76,7367.17");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5388.57,7440.60");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5961.20,7324.82");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=449.94,7104.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1112.15,7029.55");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1436.08,7243.63");
    }

}

