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

network DADCA_ACK_n20_u2_a8000m2_6
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
        @display("i=misc/sensor;p=284.97,7917.33");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=672.13,7890.87");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1606.00,7832.16");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2281.97,7997.08");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2845.54,7981.24");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3216.61,7814.53");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3716.83,7775.44");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4298.29,7978.54");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5133.19,7788.12");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5657.30,7967.58");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=960.50,7617.29");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2048.66,7618.69");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2646.64,7586.48");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5446.86,7629.29");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=520.83,7450.97");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2316.40,7347.00");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3151.53,7405.10");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3520.70,7452.91");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4004.45,7459.77");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4370.67,7355.36");
    }

}

