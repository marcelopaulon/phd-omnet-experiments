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

network DADCA_ACK_n20_u3_a8000m2_6
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
        @display("i=misc/sensor;p=496.01,7803.57");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1041.89,7775.97");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4049.98,7796.68");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4904.26,7935.49");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=227.41,7561.28");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3065.69,7516.33");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3586.92,7614.64");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4669.45,7572.98");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5486.38,7661.46");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1098.82,7336.34");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1475.41,7356.82");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2088.92,7279.85");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2454.11,7492.85");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4033.01,7317.52");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4392.38,7305.43");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5151.61,7448.49");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5571.15,7303.89");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2711.95,7034.23");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3209.29,7151.14");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3602.05,7067.78");
    }

}

