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

network DADCA_ACK_n50_u3_a8000m2_3
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
        @display("i=misc/sensor;p=160.97,7891.98");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1517.83,7753.24");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2090.27,7849.70");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3175.57,7803.44");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3718.39,7908.79");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5591.28,7930.19");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=306.53,7527.91");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1069.78,7708.44");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3963.18,7568.52");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4528.99,7513.02");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5187.53,7541.06");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=680.86,7457.94");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2724.29,7501.87");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4856.34,7321.09");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5720.58,7303.26");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1134.43,7024.91");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1722.85,7179.36");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2231.93,7220.20");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3127.87,7235.46");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3499.15,7076.38");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5346.66,7155.95");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=395.34,6962.75");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=795.59,6894.83");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2011.55,6946.52");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3972.94,6912.84");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4492.98,6855.86");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4865.07,6925.92");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5755.20,6864.64");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1449.40,6621.87");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1895.48,6554.28");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2620.57,6651.76");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3131.27,6677.57");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5241.50,6673.99");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=678.54,6367.96");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1144.07,6449.65");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2624.25,6271.74");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3632.31,6469.64");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4149.88,6489.87");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4846.21,6457.52");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1875.17,6152.80");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3242.57,6176.53");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3614.50,6078.31");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4021.29,6131.90");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5378.06,6170.74");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5779.01,6201.78");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1087.06,5941.67");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2303.54,5787.30");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2900.11,5903.11");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4606.34,6006.27");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5459.81,5828.88");
    }

}

