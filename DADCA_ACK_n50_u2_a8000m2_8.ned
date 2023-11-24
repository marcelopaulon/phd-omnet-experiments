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

network DADCA_ACK_n50_u2_a8000m2_8
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
        @display("i=misc/sensor;p=788.46,7942.80");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1445.72,7863.84");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2546.96,7867.38");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3185.97,7840.90");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4509.24,7887.95");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5174.03,7840.97");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5836.38,7784.17");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=216.03,7647.85");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2085.03,7667.04");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3643.81,7624.17");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4921.58,7586.51");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=611.36,7272.48");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1030.37,7377.37");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1554.31,7409.45");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2148.60,7318.23");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2570.51,7466.81");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5454.81,7443.61");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2878.23,7227.69");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3952.90,7253.09");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4491.18,7241.48");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4892.07,7115.99");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5723.74,7196.22");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=125.50,6998.96");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=656.36,6771.66");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1269.64,6836.05");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1689.45,6796.62");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2342.67,6975.73");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2891.17,6853.84");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3422.80,6995.36");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4443.19,6877.84");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5350.33,7009.87");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4005.19,6695.78");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5213.31,6616.90");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5614.03,6614.84");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=183.72,6385.95");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=647.23,6304.50");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1742.66,6404.03");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2256.92,6391.84");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3120.67,6376.70");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3690.86,6391.05");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4422.60,6337.17");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=85.00,6024.19");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1064.10,6222.27");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1567.13,6070.14");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2616.20,6236.22");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3971.16,6179.56");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4935.50,6065.67");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5493.81,6083.15");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5926.87,6075.39");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=453.71,5799.19");
    }

}

