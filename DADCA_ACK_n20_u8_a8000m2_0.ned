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

network DADCA_ACK_n20_u8_a8000m2_0
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
        @display("i=misc/sensor;p=2545.60,7920.34");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3496.04,7993.61");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3872.30,7925.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4228.73,7896.05");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4835.96,7997.54");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=659.91,7723.37");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1236.61,7666.82");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2182.37,7716.64");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2828.09,7651.08");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3360.95,7650.30");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3991.16,7527.89");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4561.64,7705.47");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5472.20,7748.82");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5954.60,7622.00");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=476.65,7289.37");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=919.24,7271.42");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2444.98,7473.85");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3492.48,7318.50");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4444.23,7290.76");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5125.88,7379.68");
    }

}

